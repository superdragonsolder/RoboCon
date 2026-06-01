#!/usr/bin/env python3
"""
KFS 梅花桩路径规划器
=====================
崇武探幽技能挑战赛 — 将 C++ BFS 规划器移植为 Python 模块，
可独立运行或集成到行为树任务脚本中。

用法:
    from kfs_planner import KFSPlanner

    planner = KFSPlanner()
    planner.randomize_blocks()
    actions = planner.plan_collect_2_r2()       # 全收集模式
    # 或
    actions = planner.plan_single_kfs()          # 单KFS快速模式

    for act in actions:
        print(act)  # 每条是可直接调用的 ROS2 指令
"""

import random
import heapq
from collections import deque
from typing import List, Tuple, Dict, Optional
from enum import IntEnum


# ══════════════════════════════════════════════════════════════
# 朝向枚举
# ══════════════════════════════════════════════════════════════

class Heading(IntEnum):
    North = 0  # 北 (row-1)
    East = 1   # 东 (col+1)
    South = 2  # 南 (row+1)
    West = 3   # 西 (col-1)

    @property
    def cn(self) -> str:
        return ["北", "东", "南", "西"][self.value]


# ══════════════════════════════════════════════════════════════
# 梅花桩常量
# ══════════════════════════════════════════════════════════════

# 高度表 (row 1..4, col 1..3), row=1 靠近入口, row=4 出口行
HEIGHTS: Dict[Tuple[int, int], int] = {
    (4, 1): 200, (4, 2): 400, (4, 3): 200,
    (3, 1): 400, (3, 2): 600, (3, 3): 400,
    (2, 1): 600, (2, 2): 400, (2, 3): 200,
    (1, 1): 400, (1, 2): 200, (1, 3): 400,
}

# 格子间距 (米) — 相邻梅花桩中心距离
CELL_SPACING = 1.20

# 入口/出口
ENTRY = (0, 2)           # 虚拟入口 (row=0 表示入口外)
EXITS = [(4, 1), (4, 3)]  # 两个出口

# 方向向量: N, E, S, W
DIRS = [(-1, 0), (0, 1), (1, 0), (0, -1)]


# ══════════════════════════════════════════════════════════════
# 规划器类
# ══════════════════════════════════════════════════════════════

class KFSPlanner:
    """KFS 梅花桩路径规划器。"""

    # ── 动作时间代价（秒），用于 Dijkstra 最短时间规划 ──
    COST_FORWARD_CLIMB: int = 17   # 前进一格 + 攀爬 (前进~7s + 攀爬~10s)
    COST_ROTATE_90: int = 8        # 旋转 90° (~7.5s)
    COST_GRAB: int = 6             # 夹取 (~5.5s)

    def __init__(self):
        # block: -1=空, 1=R1, 2=R2, 3=假方块
        self.block: List[List[int]] = [
            [0, 0, 0, 0, 0],
            [0, -1, -1, -1, 0],
            [0, -1, -1, -1, 0],
            [0, -1, -1, -1, 0],
            [0, -1, -1, -1, 0],
            [0, 0, 0, 0, 0],
        ]
        self.r2_positions: List[Tuple[int, int]] = []
        self.fake_pos: Optional[Tuple[int, int]] = None
        self._rng = random.Random()

    # ── 高度查询 ──

    @staticmethod
    def height_at(row: int, col: int) -> int:
        """返回 (row, col) 处的高度 (mm)，入口外 (0,*) 高度为 0。"""
        if row == 0:
            return 0
        return HEIGHTS.get((row, col), 0)

    @staticmethod
    def can_traverse_slope(from_row: int, from_col: int,
                           to_row: int, to_col: int) -> bool:
        """检查是否可以从 from 移动到 to（高度差恰好 ±200mm）。"""
        if from_row == 0:  # 从入口外进入
            return True
        return abs(KFSPlanner.height_at(to_row, to_col) -
                   KFSPlanner.height_at(from_row, from_col)) == 200

    @staticmethod
    def valid_grab_delta(dh: int) -> bool:
        """检查高度差是否允许夹取。"""
        return dh in (-200, 200, 400)

    # ── 随机布置 ──

    def randomize_blocks(self, seed: int = None) -> None:
        """随机布置 KFS：3个R1, 2个R2, 1个假方块。"""
        if seed is not None:
            self._rng = random.Random(seed)

        # 重置
        for i in range(1, 5):
            for j in range(1, 4):
                self.block[i][j] = -1

        all_cells = [(i, j) for i in range(1, 5) for j in range(1, 4)]
        forbidden_r1 = {(2, 2), (3, 2)}  # R1 不能放最中间

        # 放 3 个 R1
        candidates_r1 = [c for c in all_cells if c not in forbidden_r1]
        self._rng.shuffle(candidates_r1)
        for i in range(min(3, len(candidates_r1))):
            r, c = candidates_r1[i]
            self.block[r][c] = 1

        # 剩余位置放 2 个 R2 和 1 个假方块
        remaining = [c for c in all_cells if self.block[c[0]][c[1]] == -1]
        self._rng.shuffle(remaining)

        # 2 个 R2
        for i in range(min(2, len(remaining))):
            r, c = remaining[i]
            self.block[r][c] = 2

        # 1 个假方块（不能放出口行 (4,1)/(4,2)/(4,3)）
        if len(remaining) > 2:
            for i in range(2, len(remaining)):
                r, c = remaining[i]
                if not (r == 4 and c in (1, 2, 3)):
                    self.block[r][c] = 3
                    break

        self._refresh_cache()

    def set_blocks(self, blocks: Dict[Tuple[int, int], int]) -> None:
        """手动设置 KFS 布局。blocks[(row,col)] = 1/2/3。"""
        for i in range(1, 5):
            for j in range(1, 4):
                self.block[i][j] = blocks.get((i, j), -1)
        self._refresh_cache()

    def _refresh_cache(self) -> None:
        """刷新 R2 位置和假方块缓存。"""
        self.r2_positions = []
        self.fake_pos = None
        for i in range(1, 5):
            for j in range(1, 4):
                if self.block[i][j] == 2:
                    self.r2_positions.append((i, j))
                elif self.block[i][j] == 3:
                    self.fake_pos = (i, j)

    def print_layout(self) -> None:
        """打印当前 KFS 布局。"""
        print("\n梅花桩 KFS 布局 (行4=出口侧, 行1=入口侧):")
        print(f"      列1      列2      列3")
        labels = {1: "R1", 2: "R2", 3: "假", -1: "空"}
        for row in (4, 3, 2, 1):
            cells = []
            for col in (1, 2, 3):
                v = self.block[row][col]
                h = self.height_at(row, col)
                cells.append(f"{labels.get(v, '?')}({h}mm)")
            print(f"行{row}:  " + "  ".join(f"{c:<12}" for c in cells))

    # ── 朝向工具 ──

    @staticmethod
    def heading_from_delta(dr: int, dc: int) -> Heading:
        if dr == -1 and dc == 0: return Heading.North
        if dr == 1 and dc == 0:  return Heading.South
        if dr == 0 and dc == 1:  return Heading.East
        return Heading.West

    @staticmethod
    def rotation_steps(frm: Heading, to: Heading) -> int:
        cw = (to.value - frm.value + 4) % 4
        ccw = (frm.value - to.value + 4) % 4
        return min(cw, ccw)

    @staticmethod
    def rotation_degrees(frm: Heading, to: Heading) -> int:
        """返回旋转角度: 正=右转, 负=左转（适配本车 +Z=右转 的 convention）。"""
        cw = (to.value - frm.value + 4) % 4   # 顺时针步数
        ccw = (frm.value - to.value + 4) % 4  # 逆时针步数
        if cw <= ccw:
            return cw * 90    # 右转（CW → 正）
        else:
            return -ccw * 90  # 左转（CCW → 负）

    # ── 找到夹取 R2 所需的朝向（机器人站在 (rx,ry)，KFS 在 (kx,ky)）──

    @staticmethod
    def grab_heading(robot_pos: Tuple[int, int],
                     kfs_pos: Tuple[int, int]) -> Heading:
        """返回使 KFS 位于机器人右侧时机器人应面向的方向。"""
        rx, ry = robot_pos
        kx, ky = kfs_pos
        if kx < rx:   return Heading.West   # KFS 在北 → 面向西, KFS在右
        if kx > rx:   return Heading.East   # KFS 在南 → 面向东, KFS在右
        if ky > ry:   return Heading.North  # KFS 在东 → 面向北, KFS在右
        return Heading.South                 # KFS 在西 → 面向南, KFS在右

    # ══════════════════════════════════════════════════════════
    #  主规划：收集 2 个 R2 KFS
    # ══════════════════════════════════════════════════════════

    def plan_collect_2_r2(self) -> Optional[List[dict]]:
        """BFS 规划收集 2 个 R2 KFS 的最短路径。

        约束:
          - 只能向前移动（朝向方向），且只能向前上下坡
          - 只能夹取右侧的 KFS
          - 旋转在原地完成

        返回: List[dict] — 每条是 ROS2 动作指令，或 None（无解）。
        """
        R2s = self.r2_positions
        R = len(R2s)
        if R == 0:
            print("[规划器] 没有 R2 KFS，直接前往出口。")
            return []

        target_count = min(2, R)

        # ── 缓存 ──
        r2_idx = {pos: k for k, pos in enumerate(R2s)}
        row1_mask = 0
        is_row1 = [False] * R
        for k, (r, _) in enumerate(R2s):
            if r == 1:
                is_row1[k] = True
                row1_mask |= (1 << k)

        # ── 入口外夹取候选（第 1 行的 R2）──
        entrance_candidates = []
        for col in (1, 2, 3):
            if self.block[1][col] == 2:
                k = r2_idx.get((1, col))
                if k is not None:
                    dh = self.height_at(1, col) - 0
                    if self.valid_grab_delta(dh):
                        entrance_candidates.append(k)

        # ── 可行走检查 ──
        def is_walkable(r: int, c: int, mask: int) -> bool:
            if not (1 <= r <= 4 and 1 <= c <= 3):
                return False
            if self.block[r][c] == 3:
                return False
            if self.block[r][c] == 2:
                k = r2_idx.get((r, c))
                if k is not None and (mask & (1 << k)) == 0:
                    return False
            return True

        # ── 夹取检查：在 (r,c) 面向 h，右侧是否有可夹取的 R2 ──
        def grab_at(r: int, c: int, h: Heading, mask: int) -> int:
            """返回可新夹取的 R2 的 bitmask（最多 1 个，因为右侧只有一格）。"""
            if r == 0:
                return 0
            right_d = (h + 1) % 4
            kr, kc = r + DIRS[right_d][0], c + DIRS[right_d][1]
            if (kr, kc) in r2_idx:
                k = r2_idx[(kr, kc)]
                if not (mask & (1 << k)):
                    need_row1 = (row1_mask != 0) and ((mask & row1_mask) == 0)
                    if need_row1 and not is_row1[k]:
                        return 0
                    dh = self.height_at(kr, kc) - self.height_at(r, c)
                    if self.valid_grab_delta(dh):
                        return 1 << k
            return 0

        # ── 列2偏好 ──
        prefer_middle = (self.fake_pos is not None and self.fake_pos[1] != 2)

        # ══════════════════════════════════════════════════════
        #  Dijkstra: state = (row, col, heading, mask, mid_flag)
        #  代价按时间（秒）计算
        # ══════════════════════════════════════════════════════
        INF = 10**9
        dist = {}
        prev_state = {}
        prev_action = {}  # 'F'=前进, 'L'=左转90°, 'R'=右转90°, 'G'=夹取, 'X'=起点

        # 优先队列: (cost, state)
        pq = []

        # ── 起始状态 ──
        if row1_mask != 0:
            if not entrance_candidates:
                print("[规划器] 第1行有R2但入口外无法夹取，任务失败。")
                return None
            for k in entrance_candidates:
                mask = 1 << k
                state = (0, 2, Heading.South, mask, 0)
                dist[state] = 0
                prev_state[state] = None
                prev_action[state] = 'X'
                heapq.heappush(pq, (0, state))
        else:
            mask = 0
            for k in entrance_candidates:
                mask |= (1 << k)
            state = (0, 2, Heading.South, mask, 0)
            dist[state] = 0
            prev_state[state] = None
            prev_action[state] = 'X'
            heapq.heappush(pq, (0, state))

        best_41 = (INF, -1, None)  # cost, mid, state
        best_43 = (INF, -1, None)

        while pq:
            cd, (cr, cc, ch, cmask, cmid) = heapq.heappop(pq)
            if cd != dist.get((cr, cc, ch, cmask, cmid), INF):
                continue  # 跳过过期条目

            row1_ok = (row1_mask == 0) or ((cmask & row1_mask) != 0)
            cnt = bin(cmask).count('1')

            if cnt >= target_count and row1_ok:
                if cr == 4 and cc == 1:
                    if cd < best_41[0] or (cd == best_41[0] and prefer_middle and cmid > best_41[1]):
                        best_41 = (cd, cmid, (cr, cc, ch, cmask, cmid))
                if cr == 4 and cc == 3:
                    if cd < best_43[0] or (cd == best_43[0] and prefer_middle and cmid > best_43[1]):
                        best_43 = (cd, cmid, (cr, cc, ch, cmask, cmid))

            # ── 动作 1: 前进（仅沿朝向方向），代价 = COST_FORWARD_CLIMB ──
            if cr == 0 and cc == 2 and ch == Heading.South:
                nr, nc = 1, 2
                if is_walkable(nr, nc, cmask) and self.can_traverse_slope(cr, cc, nr, nc):
                    nmask = cmask | grab_at(nr, nc, ch, cmask)
                    nmid = cmid or (nc == 2)
                    ns = (nr, nc, ch, nmask, nmid)
                    nd = cd + self.COST_FORWARD_CLIMB
                    if nd < dist.get(ns, INF):
                        dist[ns] = nd
                        prev_state[ns] = (cr, cc, ch, cmask, cmid)
                        prev_action[ns] = 'F'
                        heapq.heappush(pq, (nd, ns))
            elif cr > 0:
                dr, dc = DIRS[ch]
                nr, nc = cr + dr, cc + dc
                if is_walkable(nr, nc, cmask) and self.can_traverse_slope(cr, cc, nr, nc):
                    nmask = cmask | grab_at(nr, nc, ch, cmask)
                    nmid = cmid or (nc == 2)
                    ns = (nr, nc, ch, nmask, nmid)
                    nd = cd + self.COST_FORWARD_CLIMB
                    if nd < dist.get(ns, INF):
                        dist[ns] = nd
                        prev_state[ns] = (cr, cc, ch, cmask, cmid)
                        prev_action[ns] = 'F'
                        heapq.heappush(pq, (nd, ns))

            # ── 动作 2: 左转 90°（原地），代价 = COST_ROTATE_90 ──
            nh = Heading((ch + 1) % 4)
            ns = (cr, cc, nh, cmask, cmid)
            nd = cd + self.COST_ROTATE_90
            if nd < dist.get(ns, INF):
                dist[ns] = nd
                prev_state[ns] = (cr, cc, ch, cmask, cmid)
                prev_action[ns] = 'L'
                heapq.heappush(pq, (nd, ns))

            # ── 动作 3: 右转 90°（原地），代价 = COST_ROTATE_90 ──
            nh = Heading((ch + 3) % 4)
            ns = (cr, cc, nh, cmask, cmid)
            nd = cd + self.COST_ROTATE_90
            if nd < dist.get(ns, INF):
                dist[ns] = nd
                prev_state[ns] = (cr, cc, ch, cmask, cmid)
                prev_action[ns] = 'R'
                heapq.heappush(pq, (nd, ns))

            # ── 动作 4: 夹取（原地，右侧），代价 = COST_GRAB ──
            if cr > 0:
                gmask = grab_at(cr, cc, ch, cmask)
                if gmask:
                    nmask = cmask | gmask
                    ns = (cr, cc, ch, nmask, cmid)
                    nd = cd + self.COST_GRAB
                    if nd < dist.get(ns, INF):
                        dist[ns] = nd
                        prev_state[ns] = (cr, cc, ch, cmask, cmid)
                        prev_action[ns] = 'G'
                        heapq.heappush(pq, (nd, ns))

        # ── 选出口 ──
        forbid_41 = (self.block[4][1] == 3)
        if forbid_41 and best_43[2] is None:
            print("[规划器] (4,1) 被假方块阻塞且无法从 (4,3) 收集 2 个 R2，任务失败。")
            return None
        if best_41[2] is None and best_43[2] is None:
            print("[规划器] 无法找到可行路径。")
            return None

        if forbid_41:
            goal = best_43[2]
        elif best_41[2] is None:
            goal = best_43[2]
        elif best_43[2] is None:
            goal = best_41[2]
        elif best_41[0] <= best_43[0]:
            goal = best_41[2]
        else:
            goal = best_43[2]

        # ── 回溯路径（完整状态序列）──
        state_seq = []
        cur = goal
        while cur is not None:
            state_seq.append(cur)
            cur = prev_state.get(cur)
        state_seq.reverse()

        total_cost = dist.get(goal, 0)
        print(f"\n[规划器] 找到路径: {len(state_seq)-1} 步, 出口 ({goal[0]},{goal[1]}), "
              f"预估耗时 {total_cost}s")
        path_str = ' → '.join(
            f'({r},{c},{h.cn})' if r > 0 else f'(入口,{h.cn})'
            for r, c, h, _, _ in state_seq
        )
        print(f"  路径: {path_str}")

        # 记录每个 R2 在哪一步（state index）被收集
        collected = {}
        for step, (_, _, _, mask, _) in enumerate(state_seq):
            for k in range(R):
                if k not in collected and (mask & (1 << k)):
                    collected[k] = step

        print("  R2 收集:")
        for k, (rx, ry) in enumerate(R2s):
            s = collected.get(k, '-')
            print(f"    R2#{k} @({rx},{ry}): step {s}")

        # ── 转换为动作列表 ──
        actions = self._path_to_actions(state_seq, R2s, collected)

        # ── 出梅林后汇合至 (5,1) ──
        exit_r, exit_c, final_heading = goal[0], goal[1], goal[2]
        actions.extend(self._go_to_rendezvous(exit_r, exit_c, final_heading))
        return actions

    def _go_to_rendezvous(self, exit_r: int, exit_c: int, heading: 'Heading') -> List[dict]:
        """从出口 (4,1) 或 (4,3) 导航至汇合点 (5,1)。"""
        actions: List[dict] = []
        cur_c = exit_c
        cur_h = heading

        print(f"\n[规划器] 出梅林，从 ({exit_r},{cur_c}) 汇合至 (5,1)")

        # Step 1: 转向南
        if cur_h != Heading.South:
            deg = self.rotation_degrees(cur_h, Heading.South)
            actions.append({
                "type": "rotate",
                "params": {"degrees": deg, "from": cur_h.cn, "to": Heading.South.cn},
                "desc": f"旋转 {deg}° ({cur_h.cn}→南) 出梅林",
            })
            cur_h = Heading.South

        # Step 2: 下台阶 → 前进 0.2m 落地到 row 5
        actions.append({
            "type": "climb",
            "params": {"cmd": 4, "delta_mm": -200},
            "desc": "下坡 -200mm，攀爬下降20cm 出梅林",
        })
        actions.append({
            "type": "forward",
            "params": {"distance": 0.2},
            "desc": f"前进 0.2m → (5,{cur_c}) 落地",
        })

        # Step 3: 如果不在列1，向西走到 (5,1)
        while cur_c > 1:
            if cur_h != Heading.West:
                deg = self.rotation_degrees(cur_h, Heading.West)
                actions.append({
                    "type": "rotate",
                    "params": {"degrees": deg, "from": cur_h.cn, "to": Heading.West.cn},
                    "desc": f"旋转 {deg}° ({cur_h.cn}→西) 转向汇合点",
                })
                cur_h = Heading.West
            actions.append({
                "type": "forward",
                "params": {"distance": CELL_SPACING},
                "desc": f"前进 {CELL_SPACING}m → (5,{cur_c-1}) 向汇合点",
            })
            cur_c -= 1

        # Step 4: 在 (5,1) 统一面朝西
        if cur_h != Heading.West:
            deg = self.rotation_degrees(cur_h, Heading.West)
            actions.append({
                "type": "rotate",
                "params": {"degrees": deg, "from": cur_h.cn, "to": Heading.West.cn},
                "desc": f"旋转 {deg}° ({cur_h.cn}→西) 面朝武馆",
            })

        return actions

    # ══════════════════════════════════════════════════════════
    #  单 KFS 快速模式
    # ══════════════════════════════════════════════════════════

    def plan_single_kfs(self) -> Optional[List[dict]]:
        """只拿 1 个最近的 R2 KFS，快速穿过梅林。"""
        R2s = self.r2_positions
        R = len(R2s)
        if R == 0:
            print("[规划器] 没有 R2 KFS。")
            return []

        def is_walkable_empty(r: int, c: int) -> bool:
            if not (1 <= r <= 4 and 1 <= c <= 3):
                return False
            if self.block[r][c] in (2, 3):
                return False
            return True

        INF = 10**9

        # BFS 从入口到所有格
        dist_from_entry = {}
        prev_from_entry = {}
        move_from_entry = {}

        q = deque()
        for ny in (1, 2, 3):
            if is_walkable_empty(1, ny):
                dist_from_entry[(1, ny)] = 1
                prev_from_entry[(1, ny)] = (0, 2)
                move_from_entry[(1, ny)] = 'E'
                q.append((1, ny))

        while q:
            cr, cc = q.popleft()
            cd = dist_from_entry[(cr, cc)]
            for d, (dr, dc) in enumerate(DIRS):
                nr, ny = cr + dr, cc + dc
                if not is_walkable_empty(nr, ny):
                    continue
                if not self.can_traverse_slope(cr, cc, nr, ny):
                    continue
                if (nr, ny) not in dist_from_entry:
                    dist_from_entry[(nr, ny)] = cd + 1
                    prev_from_entry[(nr, ny)] = (cr, cc)
                    move_from_entry[(nr, ny)] = 'WSAD'[d]
                    q.append((nr, ny))

        # BFS 从出口反向
        def bfs_from_exit(ex_r, ex_c):
            d_to = {}
            prev_to = {}
            move_to = {}
            qe = deque()
            d_to[(ex_r, ex_c)] = 0
            prev_to[(ex_r, ex_c)] = None
            qe.append((ex_r, ex_c))
            while qe:
                cr, cc = qe.popleft()
                cd = d_to[(cr, cc)]
                for d, (dr, dc) in enumerate(DIRS):
                    nr, ny = cr + dr, cc + dc
                    if not is_walkable_empty(nr, ny):
                        continue
                    # 反向检查斜率
                    if not self.can_traverse_slope(nr, ny, cr, cc):
                        continue
                    if (nr, ny) not in d_to:
                        d_to[(nr, ny)] = cd + 1
                        prev_to[(nr, ny)] = (cr, cc)
                        move_to[(nr, ny)] = 'WSAD'[(d + 2) % 4]
                        qe.append((nr, ny))
            return d_to, prev_to, move_to

        d_to_41, prev_to_41, move_to_41 = bfs_from_exit(4, 1)
        d_to_43, prev_to_43, move_to_43 = bfs_from_exit(4, 3)

        best_total = INF
        best_r2_idx = -1
        best_nx = best_ny = -1
        best_ex = best_ey = -1
        best_from_entry = False

        for k, (rx, ry) in enumerate(R2s):
            # 特殊: (1,3) 可从入口外直接夹取
            if rx == 1 and ry == 3:
                dh = self.height_at(1, 3) - 0
                if self.valid_grab_delta(dh):
                    for ey in (1, 3):
                        d_to = d_to_41 if ey == 1 else d_to_43
                        for ny in (1, 2, 3):
                            if not is_walkable_empty(1, ny):
                                continue
                            if (1, ny) not in d_to:
                                continue
                            total = 1 + d_to[(1, ny)]
                            if total < best_total:
                                best_total = total
                                best_r2_idx = k
                                best_nx, best_ny = 1, ny
                                best_ex, best_ey = 4, ey
                                best_from_entry = True

            # 正常: 到达 R2 的邻格
            for dr, dc in DIRS:
                nx, ny = rx + dr, ry + dc
                if not (1 <= nx <= 4 and 1 <= ny <= 3):
                    continue
                if not is_walkable_empty(nx, ny):
                    continue
                if (nx, ny) not in dist_from_entry:
                    continue
                dh = self.height_at(rx, ry) - self.height_at(nx, ny)
                if not self.valid_grab_delta(dh):
                    continue
                for ey in (1, 3):
                    if self.block[4][ey] == 3:
                        continue
                    d_to = d_to_41 if ey == 1 else d_to_43
                    if (nx, ny) not in d_to:
                        continue
                    total = dist_from_entry[(nx, ny)] + d_to[(nx, ny)]
                    if total < best_total:
                        best_total = total
                        best_r2_idx = k
                        best_nx, best_ny = nx, ny
                        best_ex, best_ey = 4, ey
                        best_from_entry = False

        if best_r2_idx == -1:
            print("[规划器] 单KFS模式：无法找到可行路径。")
            return None

        tr, tc = R2s[best_r2_idx]
        print(f"\n[规划器] 单KFS模式: R2 @({tr},{tc}), "
              f"收集位 ({best_nx},{best_ny}), 出口 ({best_ex},{best_ey}), "
              f"步数 {best_total}")

        # 构建路径
        path = []
        if best_from_entry:
            path.append((0, 2))
            d_to = d_to_41 if best_ey == 1 else d_to_43
            prev_to = prev_to_41 if best_ey == 1 else prev_to_43
            cr, cc = best_nx, best_ny
            while not (cr, cc) == (best_ex, best_ey):
                path.append((cr, cc))
                cr, cc = prev_to[(cr, cc)]
            path.append((best_ex, best_ey))
        else:
            # 入口→邻格
            rev = []
            cr, cc = best_nx, best_ny
            while (cr, cc) != (0, 2):
                rev.append((cr, cc))
                cr, cc = prev_from_entry[(cr, cc)]
            rev.append((0, 2))
            path = list(reversed(rev))
            # 邻格→出口
            d_to = d_to_41 if best_ey == 1 else d_to_43
            prev_to = prev_to_41 if best_ey == 1 else prev_to_43
            cr, cc = best_nx, best_ny
            while not (cr, cc) == (best_ex, best_ey):
                cr, cc = prev_to[(cr, cc)]
                path.append((cr, cc))

        print(f"  路径: {' → '.join(f'({r},{c})' for r, c in path)}")

        collected = {best_r2_idx: path.index((best_nx, best_ny))}
        return self._path_to_actions(path, R2s, collected)

    # ══════════════════════════════════════════════════════════
    #  路径 → ROS2 动作指令转换
    # ══════════════════════════════════════════════════════════

    def _path_to_actions(
        self,
        state_seq: List[Tuple[int, int, Heading, int, int]],
        r2s: List[Tuple[int, int]],
        collected: Dict[int, int],
    ) -> List[dict]:
        """将带朝向的 BFS 状态序列转换为 ROS2 动作指令列表。

        state_seq: [(row, col, heading, mask, mid_flag), ...]
        """
        actions: List[dict] = []
        r2_collected = set()

        for i in range(1, len(state_seq)):
            pr, pc, ph, pmask, _ = state_seq[i - 1]
            cr, cc, ch, cmask, _ = state_seq[i]

            # ── 入口外夹取（step 0 已收集）──
            if pr == 0 and i == 1:
                for k, step in collected.items():
                    if step == 0:
                        rx, ry = r2s[k]
                        actions.append({
                            "type": "entry_grab",
                            "params": {"kfs": (rx, ry)},
                            "desc": f"入口外夹取 R2 KFS @({rx},{ry})（机器人面向南，KFS在右侧）",
                        })
                        r2_collected.add((rx, ry))

            # ── 位置改变 → 前进（+ 坡度攀爬）──
            if (cr, cc) != (pr, pc):
                prev_h = self.height_at(pr, pc)
                cur_h = self.height_at(cr, cc)
                dh = cur_h - prev_h

                if dh == 200:
                    actions.append({
                        "type": "climb",
                        "params": {"cmd": 2, "delta_mm": 200},
                        "desc": f"上坡 +200mm ({prev_h}→{cur_h}mm)，攀爬上爬20cm",
                    })
                elif dh == -200:
                    actions.append({
                        "type": "climb",
                        "params": {"cmd": 4, "delta_mm": -200},
                        "desc": f"下坡 -200mm ({prev_h}→{cur_h}mm)，攀爬下降20cm",
                    })

                actions.append({
                    "type": "forward",
                    "params": {"distance": CELL_SPACING},
                    "desc": f"前进 {CELL_SPACING}m → ({cr},{cc}) 高度={cur_h}mm",
                })

            # ── 朝向改变 → 旋转 ──
            if ch != ph:
                deg = self.rotation_degrees(ph, ch)
                actions.append({
                    "type": "rotate",
                    "params": {"degrees": deg, "from": ph.cn, "to": ch.cn},
                    "desc": f"旋转 {deg}° ({ph.cn}→{ch.cn})",
                })

            # ── 夹取检查（mask 变化且位置/朝向不变时）──
            new_mask = cmask & ~pmask
            if new_mask and (cr, cc) == (pr, pc) and ch == ph:
                for k in range(len(r2s)):
                    if new_mask & (1 << k):
                        rx, ry = r2s[k]
                        if (rx, ry) in r2_collected:
                            continue
                        cur_h = self.height_at(cr, cc)
                        kfs_h = self.height_at(rx, ry)
                        arm_dh = kfs_h - cur_h
                        actions.append({
                            "type": "grab",
                            "params": {"kfs": (rx, ry), "delta_mm": arm_dh},
                            "desc": f"夹取 R2 KFS @({rx},{ry})，机械臂高度差={arm_dh:+d}mm",
                        })
                        r2_collected.add((rx, ry))

            # ── 夹取检查（到新位置后 mask 变了，且 KFS 在右侧）──
            elif new_mask and (cr, cc) != (pr, pc):
                for k in range(len(r2s)):
                    if new_mask & (1 << k):
                        rx, ry = r2s[k]
                        if (rx, ry) in r2_collected:
                            continue
                        # 确认 KFS 在当前位置的右侧
                        right_d = (ch + 1) % 4
                        expect_kr = cr + DIRS[right_d][0]
                        expect_kc = cc + DIRS[right_d][1]
                        if (rx, ry) != (expect_kr, expect_kc):
                            continue
                        cur_h = self.height_at(cr, cc)
                        kfs_h = self.height_at(rx, ry)
                        arm_dh = kfs_h - cur_h
                        actions.append({
                            "type": "grab",
                            "params": {"kfs": (rx, ry), "delta_mm": arm_dh},
                            "desc": f"夹取 R2 KFS @({rx},{ry})，机械臂高度差={arm_dh:+d}mm",
                        })
                        r2_collected.add((rx, ry))

        return actions

    # ══════════════════════════════════════════════════════════
    #  对抗区分析
    # ══════════════════════════════════════════════════════════

    def analyze_confrontation(self) -> str:
        """分析对抗区策略。"""
        if self.fake_pos is None:
            return "未检测到假方块。"

        fx, fy = self.fake_pos
        if fy == 2:
            return (
                f"【工况1】假KFS在中间 ({fx},{fy}) → "
                "避开中路死角，选边路放置真实KFS，利用假方块妨碍敌方。"
            )
        else:
            return (
                f"【工况2】假KFS在边侧 ({fx},{fy}) → "
                "中路敞开，抢占中轴核心格，放置中心压制位，封锁敌方快攻。"
            )


# ══════════════════════════════════════════════════════════════
# 独立测试入口
# ══════════════════════════════════════════════════════════════

if __name__ == "__main__":
    planner = KFSPlanner()
    planner.randomize_blocks(seed=42)
    planner.print_layout()

    print("\n" + "=" * 60)
    print("  全收集模式（收集 2 个 R2）")
    print("=" * 60)
    actions = planner.plan_collect_2_r2()
    if actions:
        print(f"\n动作指令序列 ({len(actions)} 条):")
        for i, act in enumerate(actions):
            print(f"  {i+1}. [{act['type']}] {act['desc']}")

    print("\n" + "=" * 60)
    print("  对抗区分析")
    print("=" * 60)
    print(planner.analyze_confrontation())
