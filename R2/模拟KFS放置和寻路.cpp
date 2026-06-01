#include<iostream>
#include<random>
#include<vector>
#include<algorithm>
#include<queue>
#include<climits>
#include<tuple>
#include<array>
#include<locale>
#include<string>

using namespace std;

enum class Heading
{
	North =0,
	East =1,
	South =2,
	West =3
};

int block[6][5] = {
{0,0 ,0 ,0 ,0 },
{0,-1, -1, -1 ,0 },
{0,-1, -1, -1 ,0 },
{0,-1, -1, -1 ,0 },
{0,-1, -1, -1 ,0 },
{0,0 ,0 ,0 ,0 }
};//记录梅花桩上方块位置1表示R1的位置，2表示R2的位置，3表示假方块的位置，0表示无方块

// 已知/观测到的假方块位置（机器人开始时不可见）
static bool knownFake[6][5] = { false };

//定义&初始化梅花桩
int box[6][5] = {
{0,0 ,0 ,0 ,0 },
{0,400,200,400,0 },
{0,200,400,600,0 },
{0,400,600,400,0 },
{0,200,400,200,0 },
{0,0 ,0 ,0 ,0 }
};

// 在所有 block == -1 的格子中随机放置方块：三个1（不能放在最中间的两个 -1），两个2，一个3
void randomize_block()
{
	// 收集所有初始为 -1 的位置（行1..4，列1..3）
	vector<pair<int, int>> all;
	for (int i =1; i <=4; ++i) {
		for (int j =1; j <=3; ++j) {
			if (block[i][j] == -1) all.emplace_back(i, j);
		}
	}

	// 定义"最中间的两个 -1"位置（按当前数组索引解释为 (2,2) 和 (3,2)）
	auto isForbidden = [](int r, int c) {
		return (r ==2 && c ==2) || (r ==3 && c ==2);
		};

	// 候选放置1 的位置（排除 forbidden）
	vector<pair<int, int>> candidatesFor1;
	for (auto& p : all) if (!isForbidden(p.first, p.second)) candidatesFor1.push_back(p);

	random_device rd;
	mt19937 gen(rd());

	// 随机选择三个位置放1（若候选不足则尽量放置）
	shuffle(candidatesFor1.begin(), candidatesFor1.end(), gen);
	int onesToPlace =3;
	if ((int)candidatesFor1.size() < onesToPlace) onesToPlace = (int)candidatesFor1.size();
	for (int k =0; k < onesToPlace; ++k) {
		block[candidatesFor1[k].first][candidatesFor1[k].second] =1;
	}

	// 剩余仍为 -1 的位置，用于放2 和3
	vector<pair<int, int>> remaining;
	for (auto& p : all) if (block[p.first][p.second] == -1) remaining.push_back(p);

	shuffle(remaining.begin(), remaining.end(), gen);
	int twosToPlace =2; // 崇武探幽挑战赛：只有 2 个 R2 KFS
	if ((int)remaining.size() < twosToPlace) twosToPlace = (int)remaining.size();
	for (int k =0; k < twosToPlace; ++k) {
		block[remaining[k].first][remaining[k].second] =2;
	}
	// 放置一个3（如果还有位置），但不能放在 (4,1),(4,2),(4,3)
	if ((int)remaining.size() > twosToPlace) {
		// 找到一个不在禁止位置的索引
		int idxFor3 = -1;
		for (int k = twosToPlace; k < (int)remaining.size(); ++k) {
			int r = remaining[k].first;
			int c = remaining[k].second;
			if (!(r ==4 && (c ==1 || c ==2 || c ==3))) {
				idxFor3 = k; break;
			}
		}
		if (idxFor3 != -1) {
			block[remaining[idxFor3].first][remaining[idxFor3].second] =3;
		}
		else {
			// 如果没有可放置的位置，跳过放置3
		}
	}
}

int x =0, y =2; // R2 入口外的虚拟起点
int h =0, lh =0, dh =0; //高度，上次高度和高度差

int height_at(int row, int col)
{
	if (row <1 || row >4 || col <1 || col >3) {
		return 0;
	}
	return box[5 - row][col];
}

const char* heading_name(Heading heading)
{
	switch (heading) {
	case Heading::North: return "北";
	case Heading::East: return "东";
	case Heading::South: return "南";
	case Heading::West: return "西";
	}
	return "未知";
}

Heading heading_from_delta(int dx, int dy)
{
	if (dx == -1 && dy ==0) return Heading::North;
	if (dx ==1 && dy ==0) return Heading::South;
	if (dx ==0 && dy ==1) return Heading::East;
	return Heading::West;
}

int rotation_steps(Heading from, Heading to)
{
	int fromValue = static_cast<int>(from);
	int toValue = static_cast<int>(to);
	int clockwise = (toValue - fromValue +4) %4;
	int counterClockwise = (fromValue - toValue +4) %4;
	return min(clockwise, counterClockwise);
}

Heading rotate_towards(Heading current, Heading target)
{
	int currentValue = static_cast<int>(current);
	int targetValue = static_cast<int>(target);
	int clockwise = (targetValue - currentValue +4) %4;
	int counterClockwise = (currentValue - targetValue +4) %4;
	if (clockwise <= counterClockwise) {
		return static_cast<Heading>((currentValue +1) %4);
	}
	return static_cast<Heading>((currentValue +3) %4);
}

bool can_traverse_slope(int currentRow, int currentCol, int nextRow, int nextCol)
{
	if (currentRow ==0) {
		return true;
	}
	return abs(height_at(nextRow, nextCol) - height_at(currentRow, currentCol)) ==200;
}

void print_motion_details(const vector<pair<int,int>>& path, const vector<pair<int,int>>& r2s)
{
	Heading heading = Heading::South;
	cout << "\n动作级执行细节：\n";
	cout << "初始朝向：" << heading_name(heading) << "（面向树林）" << endl;

	for (size_t i =1; i < path.size(); ++i) {
		int currentRow = path[i -1].first;
		int currentCol = path[i -1].second;
		int nextRow = path[i].first;
		int nextCol = path[i].second;

		if (currentRow ==0) {
			cout << "步骤 " << i << "：从入口外进入树林，目标方块 ("
				 << nextRow << "," << nextCol << ")" << endl;
			cout << "  入口侧已完成首个 KFS 拿取检查，允许进入树林。" << endl;
			continue;
		}

		int dx = nextRow - currentRow;
		int dy = nextCol - currentCol;
		Heading desiredHeading = heading_from_delta(dx, dy);
		if (heading != desiredHeading) {
			cout << "步骤 " << i << "：当前前方未对准坡面，开始原地旋转（预计 "
				 << rotation_steps(heading, desiredHeading) << " 次）。" << endl;
			while (heading != desiredHeading) {
				heading = rotate_towards(heading, desiredHeading);
				cout << "  旋转后朝向：" << heading_name(heading) << endl;
			}
		} else {
			cout << "步骤 " << i << "：当前朝向已对准前方坡面（" << heading_name(heading) << "）。" << endl;
		}

		int currentHeight = height_at(currentRow, currentCol);
		int nextHeight = height_at(nextRow, nextCol);
		int deltaHeight = nextHeight - currentHeight;
		if (abs(deltaHeight) !=200) {
			cout << "  警告：( " << currentRow << "," << currentCol << ") 到 ("
				 << nextRow << "," << nextCol << ") 的高度差为 " << deltaHeight
				 << "，不满足 ±200 的上下坡约束。" << endl;
			continue;
		}

		cout << "  当前高度=" << currentHeight << " mm，目标高度=" << nextHeight
			 << " mm，高度差=" << deltaHeight << " mm。" << endl;
		cout << "  前方激光测距：接近坡脚，距离=5 cm。" << endl;
		if (deltaHeight >0) {
			cout << "  满足条件，执行上坡命令。" << endl;
		} else {
			cout << "  满足条件，执行下坡命令。" << endl;
		}
		cout << "  完成移动，抵达 (" << nextRow << "," << nextCol << ")。" << endl;

		for (size_t k =0; k < r2s.size(); ++k) {
			if ((r2s[k].first == currentRow && abs(r2s[k].second - currentCol) ==1) ||
				(r2s[k].second == currentCol && abs(r2s[k].first - currentRow) ==1)) {
				int kx = r2s[k].first, ky = r2s[k].second;
				Heading grabHeading;
				if (kx < currentRow) grabHeading = Heading::East;
				else if (kx > currentRow) grabHeading = Heading::West;
				else if (ky > currentCol) grabHeading = Heading::South;
				else grabHeading = Heading::North;

				if (heading != grabHeading) {
					cout << "  [旋转预备] 目标 KFS 不在左侧！当前朝向：" << heading_name(heading)
						 << "，需原地旋转至：" << heading_name(grabHeading) << "，使 KFS 置于左臂侧。" << endl;
					heading = grabHeading;
				}

				int robot_h = height_at(currentRow, currentCol);
				int target_h = height_at(kx, ky);
				int dh = target_h - robot_h;
				string armAction = "";
				if (dh == -200) armAction = "机械臂向下伸取 (-200mm)";
				else if (dh == 200) armAction = "机械臂向上抬起 (+200mm)";
				else if (dh == 400) armAction = "机械臂大幅向上抬起 (+400mm)";
				else armAction = "不可操作高度，放弃夹取 (" + to_string(dh) + "mm)";

				cout << "  [执行动作] " << armAction << "，正在拿取左侧的 R2 KFS @(" << kx << "," << ky << ")。" << endl;
			}
		}
	}
}

//定义前进、后退、左移、右移函数
void move_forward(int a, int /*b*/)
{
	if (a +1 <=4)
	{
		x = a +1;
		cout << "当前位置" << x << " " << y << "当前高度：" << box[5 - x][y] << endl;
	}
	else
	{
		cout << "无法前进，已到达边界！" << endl;
	}
}
void move_backward(int a, int /*b*/)
{
	if (a -1 >=1)
	{
		x = a -1;
		cout << "当前位置" << x << " " << y << "当前高度：" << box[5 - x][y] << endl;
	}
	else
	{
		cout << "无法后退，已到达边界！" << endl;
	}
}
void move_left(int /*a*/, int b)
{
	if (b -1 >=1)
	{
		y = b -1;
		cout << "当前位置" << x << " " << y << "当前高度：" << box[5 - x][y] << endl;
	}
	else
	{
		cout << "无法左移，已到达边界！" << endl;
	}
}
void move_right(int /*a*/, int b)
{
	if (b +1 <=3)
	{
		y = b +1;
		cout << "当前位置" << x << " " << y << "当前高度：" << box[5 - x][y] << endl;
	}
	else
	{
		cout << "无法右移，已到达边界！" << endl;
	}
}
void print_location()
{
	if (x ==0)
	{
		cout << "当前位置：R2入口外 (0,2)，尚未登上树林方块" << endl;
		return;
	}

	//打印我当前位于整个梅花桩的哪里
	int arr[5][4] = {0 };
	arr[x][y] =9;
	for (int i =4; i >=1; i--)
	{
		for (int j =1; j <=3; j++)
		{
			if (arr[i][j] ==9)
				cout << "我" << "\t";
			else
				cout << arr[i][j] << "\t";
		}
		cout << endl;
	}
}

// BFS solver:
// 1) R2 不能走进任何仍然带有 KFS 的格子，只能站在空格上从相邻位置拿取 R2 KFS。
// 2) 如果第一行 (1,1)(1,2)(1,3) 上有 R2 KFS，则第一次夹取必须在入口外完成（需满足左侧朝向限制）。
// 3) 只收集 2 个 R2 KFS，且路径为最短；若假 KFS 不在中间列，则优先经过列2。
void solve_and_print_path()
{
	// collect R2 coordinates
	vector<pair<int,int>> r2s;
	int r2Index[6][5];
	for (int i =0; i <6; ++i) {
		for (int j =0; j <5; ++j) {
			r2Index[i][j] = -1;
		}
	}
	for (int i =1; i <=4; ++i) {
		for (int j =1; j <=3; ++j) {
			if (block[i][j] ==2) {
				r2Index[i][j] = static_cast<int>(r2s.size());
				r2s.emplace_back(i,j);
			}
		}
	}
	int R = (int)r2s.size();
	if (R ==0) {
		cout << "没有 R2，直接前往目标。" << endl;
		return;
	}
	if (R >4) R =4; // safety

	int fake_x = -1, fake_y = -1;
	for (int i =1; i <=4; ++i) {
		for (int j =1; j <=3; ++j) {
			if (block[i][j] ==3) { fake_x = i; fake_y = j; }
		}
	}
	bool preferMiddle = (fake_y != -1 && fake_y != 2);
	int targetCount = min(2, R);

	vector<bool> isRow1(R, false);
	int row1Mask = 0;
	for (int k =0; k < R; ++k) {
		if (r2s[k].first == 1) {
			isRow1[k] = true;
			row1Mask |= (1<<k);
		}
	}

	auto valid_height_delta = [&](int dh) -> bool {
		return (dh == -200 || dh == 200 || dh == 400);
	};

	// coverage sets: for each R2, which positions can collect it
	vector<vector<pair<int,int>>> cover(R);
	for (int k =0; k < R; ++k) {
		int rx = r2s[k].first, ry = r2s[k].second;
		int dx[4] = {-1, 1, 0, 0};
		int dy[4] = {0, 0, -1, 1};
		for (int d = 0; d < 4; ++d) {
			int nx = rx + dx[d], ny = ry + dy[d];
			if (nx < 1 || nx > 4 || ny < 1 || ny > 3) continue;
			int dh = height_at(rx, ry) - height_at(nx, ny);
			if (valid_height_delta(dh)) {
				cover[k].push_back({nx, ny});
			}
		}
	}

	auto entrance_heading_ok = [&](int row, int col, Heading& req)->bool {
		int dx = row - 0;
		int dy = col - 2;
		Heading heads[4] = {Heading::North, Heading::East, Heading::South, Heading::West};
		for (Heading h : heads) {
			int lx =0, ly =0;
			switch (h) {
			case Heading::North: lx = 0; ly = -1; break;
			case Heading::East:  lx = -1; ly = 0; break;
			case Heading::South: lx = 0; ly = 1; break;
			case Heading::West:  lx = 1; ly = 0; break;
			}
			// 严格左侧：KFS 必须正好位于机器人左侧相邻格
			if (dx == lx && dy == ly) {
				req = h;
				return true;
			}
		}
		return false;
	};

	// 入口外可直接夹取第一行目标 (1,1) / (1,2) / (1,3)。
	vector<int> entranceCandidates;
	for (int col = 1; col <= 3; ++col) {
		if (block[1][col] == 2) {
			int idx = r2Index[1][col];
			if (idx != -1) {
				Heading reqHeading = Heading::South;
				if (!entrance_heading_ok(1, col, reqHeading)) {
					continue;
				}
				int dh = height_at(1, col) - 0; // 虚拟入口点 (0,2) 高度为0
				if (valid_height_delta(dh)) {
					entranceCandidates.push_back(idx);
				}
			}
		}
	}

	auto mask_at = [&](int i, int j, int currentMask)->int{
		int m =0;
		bool needRow1First = (row1Mask != 0) && ((currentMask & row1Mask) == 0);
		for (int k =0; k < R; ++k) {
			if (currentMask & (1<<k)) continue;
			if (needRow1First && !isRow1[k]) continue;
			for (auto &p : cover[k]) {
				if (p.first==i && p.second==j) {
					m |= (1<<k);
					break;
				}
			}
		}
		return m;
	};

	auto is_walkable = [&](int i, int j, int currentMask)->bool {
		if (i <1 || i >4 || j <1 || j >3) return false;
		if (block[i][j] ==3) return false;
		if (block[i][j] ==2) {
			int idx = r2Index[i][j];
			if (idx != -1 && (currentMask & (1<<idx)) ==0) {
				return false;
			}
		}
		return true;
	};

	auto count_bits = [&](int mask)->int {
		int cnt =0;
		while (mask) { cnt += (mask & 1); mask >>= 1; }
		return cnt;
	};

	// BFS state: x,y,mask,midFlag，其中 (0,2) 表示 R2 入口外的虚拟起点。
	static int dist[6][4][1<<4][2];
	static tuple<int,int,int,int> prevState[6][4][1<<4][2];
	static char prevMove[6][4][1<<4][2];
	for (int i =0; i <6; ++i) {
		for (int j =0; j <4; ++j) {
			for (int m =0; m < (1<<R); ++m) {
				for (int f =0; f <2; ++f) {
					dist[i][j][m][f] = -1;
				}
			}
		}
	}

	queue<tuple<int,int,int,int>> q; // x,y,mask,midFlag
	const int sx =0;
	const int sy =2;
	if (row1Mask != 0) {
		if (entranceCandidates.empty()) {
			cout << "第一行存在 R2 KFS，但入口外无法完成夹取，任务失败。" << endl;
			return;
		}
		cout << "第一行存在 R2 KFS，必须在入口外完成首次夹取（满足左侧朝向限制）。" << endl;
		for (int idx : entranceCandidates) {
			int startMask = (1<<idx);
			if (dist[sx][sy][startMask][0] == -1) {
				dist[sx][sy][startMask][0] = 0;
				prevState[sx][sy][startMask][0]=make_tuple(-1,-1,-1,-1);
				prevMove[sx][sy][startMask][0]='X';
				q.emplace(sx,sy,startMask,0);
			}
		}
	} else {
		if (!entranceCandidates.empty()) {
			cout << "入口侧存在第一行 R2 KFS，可从入口外直接夹取（优先第一行）。" << endl;
			for (int idx : entranceCandidates) {
				int startMask = (1<<idx);
				if (dist[sx][sy][startMask][0] == -1) {
					dist[sx][sy][startMask][0] = 0;
					prevState[sx][sy][startMask][0]=make_tuple(-1,-1,-1,-1);
					prevMove[sx][sy][startMask][0]='X';
					q.emplace(sx,sy,startMask,0);
				}
			}
		} else {
			dist[sx][sy][0][0] = 0;
			prevState[sx][sy][0][0]=make_tuple(-1,-1,-1,-1);
			prevMove[sx][sy][0][0]='X';
			q.emplace(sx,sy,0,0);
		}
	}

	// track best goal for each exit separately
	int bestDepth41 = -1; int bestMid41 = -1; tuple<int,int,int,int> bestState41;
	int bestDepth43 = -1; int bestMid43 = -1; tuple<int,int,int,int> bestState43;

	auto is_better = [&](int depth, int midFlag, int bestDepth, int bestMid)->bool {
		if (bestDepth == -1 || depth < bestDepth) return true;
		if (depth > bestDepth) return false;
		if (preferMiddle) return midFlag > bestMid;
		return false;
	};

	while(!q.empty()){
		auto [cx,cy,cmask,cmid] = q.front(); q.pop();
		int cd = dist[cx][cy][cmask][cmid];
		// check if this is a goal (collected 2 R2 KFS)
		bool row1Ok = (row1Mask == 0) || ((cmask & row1Mask) != 0);
		if (count_bits(cmask) == targetCount && row1Ok) {
			if (cx==4 && cy==1 && is_better(cd, cmid, bestDepth41, bestMid41)) {
				bestDepth41 = cd; bestMid41 = cmid; bestState41 = make_tuple(cx,cy,cmask,cmid);
			}
			if (cx==4 && cy==3 && is_better(cd, cmid, bestDepth43, bestMid43)) {
				bestDepth43 = cd; bestMid43 = cmid; bestState43 = make_tuple(cx,cy,cmask,cmid);
			}
		}

		if (cx ==0 && cy ==2) {
			for (int ny =1; ny <=3; ++ny) {
				int nx =1;
				if (!is_walkable(nx, ny, cmask)) continue;
				int nmask = cmask | mask_at(nx, ny, cmask);
				int nmid = cmid || (ny == 2);
				if (dist[nx][ny][nmask][nmid] == -1) {
					dist[nx][ny][nmask][nmid] = cd + 1;
					prevState[nx][ny][nmask][nmid]=make_tuple(cx,cy,cmask,cmid);
					prevMove[nx][ny][nmask][nmid]='E';
					q.emplace(nx,ny,nmask,nmid);
				}
			}
			continue;
		}

		const int dx[4] = {-1,0,1,0};
		const int dy[4] = {0,-1,0,1};
		const char mv[4] = {'W','A','S','D'};
		for (int d=0; d<4; ++d) {
			int nx = cx + dx[d], ny = cy + dy[d];
			if (!is_walkable(nx, ny, cmask)) continue;
			if (!can_traverse_slope(cx, cy, nx, ny)) continue;
			int nmask = cmask | mask_at(nx, ny, cmask);
			int nmid = cmid || (ny == 2);
			if (dist[nx][ny][nmask][nmid] == -1){
				dist[nx][ny][nmask][nmid] = cd + 1;
				prevState[nx][ny][nmask][nmid]=make_tuple(cx,cy,cmask,cmid);
				prevMove[nx][ny][nmask][nmid]=mv[d];
				q.emplace(nx,ny,nmask,nmid);
			}
		}
	}

	// decide which exit to use based on fake block at (4,1)
	bool forbid41 = (block[4][1] ==3);

	tuple<int,int,int,int> goalState;
	if (forbid41) {
		if (bestDepth43 == -1) {
			cout << "由于 (4,1) 被假方块阻塞，且无法从 (4,3) 收集到 2 个 R2，任务失败。" << endl;
			return;
		}
		goalState = bestState43;
	} else {
		if (bestDepth41 == -1 && bestDepth43 == -1) {
			cout << "无法找到满足条件的路径（无法收集 2 个 R2 或到达任一出口）。" << endl;
			return;
		} else if (bestDepth41 == -1) {
			goalState = bestState43;
		} else if (bestDepth43 == -1) {
			goalState = bestState41;
		} else {
			goalState = (bestDepth41 <= bestDepth43) ? bestState41 : bestState43;
		}
	}

	// reconstruct path from chosen goalState
	int gx = get<0>(goalState), gy = get<1>(goalState), gmask = get<2>(goalState), gmid = get<3>(goalState);
	vector<pair<int,int>> path;
	vector<tuple<int,int,int>> statePath;
	vector<char> moves;
	int cx2 = gx, cy2 = gy, cm2 = gmask, cf2 = gmid;
	while (true) {
		auto p = prevState[cx2][cy2][cm2][cf2];
		int px = get<0>(p), py = get<1>(p), pm = get<2>(p), pf = get<3>(p);
		char mv = prevMove[cx2][cy2][cm2][cf2];
		statePath.emplace_back(cx2,cy2,cm2);
		path.emplace_back(cx2,cy2);
		if (mv!='X') moves.push_back(mv);
		if (px==-1) break;
		cx2=px; cy2=py; cm2=pm; cf2=pf;
	}
	reverse(path.begin(), path.end());
	reverse(statePath.begin(), statePath.end());

	moves.clear();
	for (size_t i =1; i < path.size(); ++i) {
		int px = path[i-1].first, py = path[i-1].second;
		int nx = path[i].first, ny = path[i].second;
		int dx = nx - px;
		int dy = ny - py;
		if (dx == -1 && dy ==0) moves.push_back('W');
		else if (dx ==1 && dy ==0) moves.push_back('S');
		else if (dx ==0 && dy == -1) moves.push_back('A');
		else if (dx ==0 && dy ==1) moves.push_back('D');
		else moves.push_back('?');
	}

	cout << "找到最短路径（收集" << targetCount << "个 R2），步数=" << ((int)path.size()-1) << "。\n";
	if (preferMiddle) {
		cout << "中路优先：" << (gmid ? "已通过列2" : "未通过列2") << "。\n";
	}
	cout << "目标出口： ("<<gx<<","<<gy<<")\n";
	cout << "路径坐标： ";
	for (auto &p : path) cout << "("<<p.first<<","<<p.second<<") ";
	cout << "\n";

	for (size_t i =0; i < path.size(); ++i) {
		if (path[i].first ==0) {
			cout << "步 " << i << ": 入口外 (0,2)\n";
		} else {
			cout << "步 " << i << ": (" << path[i].first << "," << path[i].second << ")\n";
		}
		x = path[i].first;
		y = path[i].second;
		print_location();
	}

	vector<int> collectedAt(R, -1);
	for (int step=0; step < (int)statePath.size(); ++step) {
		int stateMask = get<2>(statePath[step]);
		for (int k =0; k < R; ++k) {
			if (collectedAt[k] == -1 && (stateMask & (1<<k))) {
				collectedAt[k] = step;
			}
		}
	}
	for (int k=0;k<R;++k){
		cout << "R2 #"<<k<<" at ("<<r2s[k].first<<","<<r2s[k].second<<") collected at step ";
		if (collectedAt[k]==-1) cout << "-"; else cout << collectedAt[k];
		cout << "\n";
	}

	print_motion_details(path, r2s);
}

// ============================================================================
// 单 KFS 模式：R2 只能携带 1 个 R2 KFS，穿过梅林只需拿 1 个最近的
// 策略：入口 → 最近可收集的 R2 相邻格 → 最近出口
// ============================================================================
void solve_single_kfs_path()
{
	vector<pair<int,int>> r2s;
	for (int i =1; i <=4; ++i) {
		for (int j =1; j <=3; ++j) {
			if (block[i][j] ==2) r2s.emplace_back(i,j);
		}
	}
	int R = (int)r2s.size();
	if (R ==0) {
		cout << "没有 R2 KFS，直接前往出口。" << endl;
		return;
	}

	auto valid_height_delta = [&](int dh) -> bool {
		return (dh == -200 || dh == 200 || dh == 400);
	};

	auto is_walkable_empty = [&](int i, int j) -> bool {
		if (i <1 || i >4 || j <1 || j >3) return false;
		if (block[i][j] ==3) return false;  // 假方块不可走
		if (block[i][j] ==2) return false;  // 所有 R2 格不可走
		return true;  // R1 和空格可走
	};

	const int INF = 9999;
	int dist_from_entry[6][5];
	tuple<int,int> prev_from_entry[6][5];
	char move_from_entry[6][5];
	for (int i=0;i<6;++i) for (int j=0;j<5;++j) {
		dist_from_entry[i][j]=INF;
		prev_from_entry[i][j]=make_tuple(-1,-1);
		move_from_entry[i][j]='X';
	}

	queue<tuple<int,int,int>> q;
	dist_from_entry[0][2]=0;
	prev_from_entry[0][2]=make_tuple(-1,-1);
	move_from_entry[0][2]='X';
	for (int ny=1; ny<=3; ++ny) {
		if (is_walkable_empty(1, ny)) {
			dist_from_entry[1][ny]=1;
			prev_from_entry[1][ny]=make_tuple(0,2);
			move_from_entry[1][ny]='E';
			q.emplace(1,ny,1);
		}
	}

	const int dx4[4]={-1,0,1,0};
	const int dy4[4]={0,-1,0,1};
	const char mv4[5]="WASD";
	while (!q.empty()) {
		auto [cx,cy,cd]=q.front(); q.pop();
		for (int d=0;d<4;++d) {
			int nx=cx+dx4[d], ny=cy+dy4[d];
			if (!is_walkable_empty(nx,ny)) continue;
			if (!can_traverse_slope(cx,cy,nx,ny)) continue;
			if (dist_from_entry[nx][ny]==INF) {
				dist_from_entry[nx][ny]=cd+1;
				prev_from_entry[nx][ny]=make_tuple(cx,cy);
				move_from_entry[nx][ny]=mv4[d];
				q.emplace(nx,ny,cd+1);
			}
		}
	}

	int dist_to_41[6][5]; tuple<int,int> prev_to_41[6][5]; char move_to_41[6][5];
	int dist_to_43[6][5]; tuple<int,int> prev_to_43[6][5]; char move_to_43[6][5];

	for (int i=0;i<6;++i) for (int j=0;j<5;++j) {
		dist_to_41[i][j]=INF; prev_to_41[i][j]=make_tuple(-1,-1); move_to_41[i][j]='X';
		dist_to_43[i][j]=INF; prev_to_43[i][j]=make_tuple(-1,-1); move_to_43[i][j]='X';
	}

	{
		queue<tuple<int,int,int>> qe;
		dist_to_41[4][1]=0; qe.emplace(4,1,0);
		while (!qe.empty()) {
			auto [cx,cy,cd]=qe.front(); qe.pop();
			for (int d=0;d<4;++d) {
				int nx=cx+dx4[d], ny=cy+dy4[d];
				if (!is_walkable_empty(nx,ny)) continue;
				if (!can_traverse_slope(nx,ny,cx,cy)) continue;
				if (dist_to_41[nx][ny]==INF) {
					dist_to_41[nx][ny]=cd+1;
					prev_to_41[nx][ny]=make_tuple(cx,cy);
					if (d==0) move_to_41[nx][ny]='S';
					else if (d==1) move_to_41[nx][ny]='D';
					else if (d==2) move_to_41[nx][ny]='W';
					else move_to_41[nx][ny]='A';
					qe.emplace(nx,ny,cd+1);
				}
			}
		}
	}

	{
		queue<tuple<int,int,int>> qe;
		dist_to_43[4][3]=0; qe.emplace(4,3,0);
		while (!qe.empty()) {
			auto [cx,cy,cd]=qe.front(); qe.pop();
			for (int d=0;d<4;++d) {
				int nx=cx+dx4[d], ny=cy+dy4[d];
				if (!is_walkable_empty(nx,ny)) continue;
				if (!can_traverse_slope(nx,ny,cx,cy)) continue;
				if (dist_to_43[nx][ny]==INF) {
					dist_to_43[nx][ny]=cd+1;
					prev_to_43[nx][ny]=make_tuple(cx,cy);
					if (d==0) move_to_43[nx][ny]='S';
					else if (d==1) move_to_43[nx][ny]='D';
					else if (d==2) move_to_43[nx][ny]='W';
					else move_to_43[nx][ny]='A';
					qe.emplace(nx,ny,cd+1);
				}
			}
		}
	}

	int best_total = INF;
	int best_r2_idx = -1;
	int best_nx = -1, best_ny = -1;
	int best_ex = -1, best_ey = -1;
	bool best_from_entry_side = false;

	for (int k=0; k<R; ++k) {
		int rx=r2s[k].first, ry=r2s[k].second;

		if (rx==1 && ry==3) {
			int dh = height_at(1, 3) - 0;
			if (valid_height_delta(dh)) {
				for (int ex_val : {41, 43}) {
					int exx = (ex_val==41) ? 4 : 4;
					int exy = (ex_val==41) ? 1 : 3;
					auto& d_to = (ex_val==41) ? dist_to_41 : dist_to_43;
					for (int ny=1; ny<=3; ++ny) {
						if (!is_walkable_empty(1, ny)) continue;
						if (d_to[1][ny]==INF) continue;
						int total = 1 + d_to[1][ny];
						if (total < best_total) {
							best_total = total;
							best_r2_idx = k;
							best_nx = 1; best_ny = ny;
							best_ex = exx; best_ey = exy;
							best_from_entry_side = true;
						}
					}
				}
			}
		}

		int dx[4] = {-1, 1, 0, 0};
		int dy[4] = {0, 0, -1, 1};
		for (int d = 0; d < 4; ++d) {
			int nx = rx + dx[d], ny = ry + dy[d];
			if (nx < 1 || nx > 4 || ny < 1 || ny > 3) continue;
			if (!is_walkable_empty(nx, ny) || dist_from_entry[nx][ny] == INF) continue;
			int dh = height_at(rx, ry) - height_at(nx, ny);
			if (!valid_height_delta(dh)) continue;
			for (int ex_val : {41, 43}) {
				int exx=4;
				int exy=(ex_val==41)?1:3;
				auto& d_to = (ex_val==41) ? dist_to_41 : dist_to_43;
				if (block[exx][exy]==3) continue;
				if (d_to[nx][ny]==INF) continue;
				int total = dist_from_entry[nx][ny] + d_to[nx][ny];
				if (total < best_total) {
					best_total = total;
					best_r2_idx = k;
					best_nx = nx; best_ny = ny;
					best_ex = exx; best_ey = exy;
					best_from_entry_side = false;
				}
			}
		}
	}

	if (best_r2_idx==-1) {
		cout << "无法找到路径：没有 R2 KFS 可从入口到达并走到出口。" << endl;
		return;
	}

	int tr = r2s[best_r2_idx].first, tc = r2s[best_r2_idx].second;

	cout << "\n========== 单 KFS 模式 ==========" << endl;
	cout << "R2 只能携带 1 个 KFS，策略：拿最近的 1 个然后离开梅林。" << endl;
	cout << "\n选择目标 R2 KFS @(" << tr << "," << tc << ")" << endl;
	cout << "收集位置（相邻格）：(" << best_nx << "," << best_ny << ")" << endl;
	cout << "出口：(" << best_ex << "," << best_ey << ")" << endl;
	cout << "总步数：" << best_total << endl;

	vector<pair<int,int>> path;
	vector<char> moves;

	if (best_from_entry_side) {
		path.emplace_back(0,2);
		moves.push_back('E');

		auto& prev_to = (best_ey==1) ? prev_to_41 : prev_to_43;
		auto& move_to = (best_ey==1) ? move_to_41 : move_to_43;
		vector<pair<int,int>> second_half;
		vector<char> second_moves;
		int cx=best_nx, cy=best_ny;
		while (!(cx==best_ex && cy==best_ey)) {
			second_half.emplace_back(cx,cy);
			auto [px,py]=prev_to[cx][cy];
			second_moves.push_back(move_to[cx][cy]);
			cx=px; cy=py;
		}
		second_half.emplace_back(best_ex, best_ey);
		for (size_t i=0; i<second_half.size(); ++i) {
			path.push_back(second_half[i]);
			if (i<second_moves.size()) moves.push_back(second_moves[i]);
		}
	} else {
		vector<pair<int,int>> first_half;
		vector<char> first_moves;
		int cx=best_nx, cy=best_ny;
		while (!(cx==0 && cy==2)) {
			first_half.emplace_back(cx,cy);
			auto [px,py]=prev_from_entry[cx][cy];
			first_moves.push_back(move_from_entry[cx][cy]);
			cx=px; cy=py;
		}
		first_half.emplace_back(0,2);
		reverse(first_half.begin(), first_half.end());
		reverse(first_moves.begin(), first_moves.end());

		auto& prev_to = (best_ey==1) ? prev_to_41 : prev_to_43;
		auto& move_to = (best_ey==1) ? move_to_41 : move_to_43;
		vector<pair<int,int>> second_half;
		vector<char> second_moves;
		cx=best_nx; cy=best_ny;
		while (!(cx==best_ex && cy==best_ey)) {
			auto [px,py]=prev_to[cx][cy];
			second_moves.push_back(move_to[cx][cy]);
			cx=px; cy=py;
			second_half.emplace_back(cx,cy);
		}

		for (auto& p : first_half) path.push_back(p);
		for (char m : first_moves) moves.push_back(m);
		for (size_t i=0; i<second_half.size(); ++i) {
			path.push_back(second_half[i]);
			if (i<second_moves.size()) moves.push_back(second_moves[i]);
		}
	}

	cout << "\n路径坐标： ";
	for (auto& p : path) cout << "(" << p.first << "," << p.second << ") ";
	cout << "\n移动指令： ";
	for (char m : moves) {
		switch (m) {
			case 'W': cout << "↑ "; break;
			case 'S': cout << "↓ "; break;
			case 'A': cout << "← "; break;
			case 'D': cout << "→ "; break;
			case 'E': cout << "[入] "; break;
			default: cout << "? "; break;
		}
	}
	cout << endl;

	cout << "\n逐步演示：\n";
	for (size_t i=0; i<path.size(); ++i) {
		if (path[i].first==0) {
			cout << "步 " << i << ": 入口外 (0,2)";
			if (best_from_entry_side && i==0) {
				cout << " [在入口外拿取 R2 KFS @(" << tr << "," << tc << ")]";
			}
			cout << endl;
		} else {
			cout << "步 " << i << ": (" << path[i].first << "," << path[i].second << ")";
			if (!best_from_entry_side &&
				abs(path[i].first-tr) + abs(path[i].second-tc) == 1) {
				cout << " [在此拿取 R2 KFS @(" << tr << "," << tc << ")]";
			}
			cout << endl;
		}
		x = path[i].first;
		y = path[i].second;
		print_location();
	}

	cout << "\n✓ 单 KFS 收集完成！携带 1 个 R2 KFS 从出口 ("
	     << best_ex << "," << best_ey << ") 离开梅林，进入对抗区。" << endl;
}

// ============================================================================
// 阶段三：对抗区(CF) 策略分析
// ============================================================================
void analyze_confrontation()
{
	cout << "\n========== 阶段三：对抗区(CF) 策略分析 ==========" << endl;
	int fake_x = -1, fake_y = -1;
	for (int i = 1; i <= 4; ++i) {
		for (int j = 1; j <= 3; ++j) {
			if (block[i][j] == 3) {
				fake_x = i;
				fake_y = j;
				break;
			}
		}
	}

	if (fake_x == -1) {
		cout << "未检测到假方块(Fake KFS)。" << endl;
		return;
	}

	cout << "查得 假 KFS 坐标：(" << fake_x << "," << fake_y << ")" << endl;
	bool is_middle = (fake_y == 2);

	if (is_middle) {
		cout << ">>>【工况 1】假 KFS 在梅林中间 <<<" << endl;
		cout << "  [局势] 假方块位于中心通道 (列2)，形成一定的视线干扰或物理路线阻挡。" << endl;
		cout << "  [执行建议] R2 进入对抗区后：" << endl;
		cout << "      1. 尽量避开受假方块影响的中路死角；" << endl;
		cout << "      2. 优先选择己方安全的一道边路（列1 或 列3）纵深放置真实 R2 KFS；" << endl;
		cout << "      3. 组织边路防线，同时利用该假方块妨碍敌方判断/推进。" << endl;
	} else {
		cout << ">>>【工况 2】假 KFS 不在梅林中间 <<<" << endl;
		cout << "  [局势] 假方块位于边侧 (当前列 " << fake_y << ")，中路 (列2) 完全敞开。" << endl;
		cout << "  [执行建议] R2 进入对抗区后：" << endl;
		cout << "      1. 趁中路空虚，快速抢占对抗区中轴核心格；" << endl;
		cout << "      2. 将携带的真实 R2 KFS 放置于中心压制位；" << endl;
		cout << "      3. 封锁敌方正面快攻的最佳线路。" << endl;
	}
}

int main()
{
	// 设置区域为系统默认，支持中文输出
	setlocale(LC_ALL, "");
	cout.imbue(locale(""));
	
	cout << "欢迎使用梅花桩控制系统！" << endl;
	cout << "R2 从入口外 (0,2) 出发，首次进入树林前可在入口完成第一次拿取。" << endl;

	// 选择模式
	int mode = 0;
	cout << "\n请选择路径规划模式：" << endl;
	cout << "  1 - 全收集模式（收集 2 个 R2 KFS 后离开）" << endl;
	cout << "  2 - 单 KFS 模式（只拿 1 个最近的 R2 KFS，快速穿过梅林）" << endl;
	cout << "请输入 (1/2): ";
	cin >> mode;

	// 在程序开始时随机分配 block
	randomize_block();

	// 可选：打印 block 分布，便于调试
	cout << "block 初始分布（行1..4 列1..3，0表示非格点）：\n";
	for (int i =4; i >=1; --i) {
		for (int j =1; j <=3; ++j) {
			if (block[i][j] == -1)
				cout << "空" << "\t";
			else if (block[i][j] ==1)
				cout << "R1" << "\t";
			else if (block[i][j] ==2)
				cout << "R2" << "\t";
			else
				cout << "X" << "\t"; // 假方块用 X 标记
		}
		cout << "\n";
	}

	// call solver to compute and print optimal path
	if (mode == 2) {
		solve_single_kfs_path();
	} else {
		solve_and_print_path();
	}

	// 进行最终的对抗区出后策略分析
	analyze_confrontation();

	return 0;
}