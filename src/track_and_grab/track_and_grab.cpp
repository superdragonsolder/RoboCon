#include<iostream>
#include<random>
#include<vector>
#include<algorithm>
#include<queue>
#include<climits>
#include<tuple>
#include<cmath>
#include<array>
#include<iomanip>
#include<clocale>

using namespace std;

// 全局物理参数（可调整）
constexpr double CELL_SIZE = 1.0; // 每格大小（米）
constexpr double HALF_ROBOT = 0.4; // 车半边长度（米），车体为 0.8m 正方形
// 机器人朝向：0 = front(向上), 1 = right, 2 = back(向下), 3 = left
int dir = 0; // 初始朝向：面向前方（向上）

// 机器人连续位姿（米），以场地左上角为原点，X 横向向右，Y 纵向向下
double posX = 0.0, posY = 0.0; // 将在执行前基于整数格子初始化

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

// 在所有 block == -1 的格子中随机放置方块：三个1（不能放在最中间的两个 -1），四个2，一个3
void randomize_block()
{
	// 收集所有初始为 -1 的位置（行1..4，列1..3）
	vector<pair<int, int>> all;
	for (int i =1; i <=4; ++i) {
		for (int j =1; j <=3; ++j) {
			if (block[i][j] == -1) all.emplace_back(i, j);
		}
	}

	// 定义“最中间的两个 -1”位置（按当前数组索引解释为 (2,2) 和 (3,2)）
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
	int twosToPlace =4;
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

int x =1, y =2; //初始位置 (changed to row4, col2)
int h =0, lh =0, dh =0; //高度，上次高度和高度差

//定义前进、后退、左移、右移函数
void move_forward(int a, int b)
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
void move_backward(int a, int b)
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
void move_left(int a, int b)
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
void move_right(int a, int b)
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

// 四个方向的激光测距仪仿真（位于机器人方格中心各边的中点）
// 输出顺序（按顺时针）：front (向上), right (向右), back (向下), left (向左)
// 坐标系：整场左上角为原点，X 向右为横轴，Y 向下为纵轴；格子大小假定为 cell_size 米。
// 车体为 0.8m 的正方形，传感器位于每边中心，距车中心偏移 half_size = 0.4m。
// 计算方法：从传感器表面沿轴线逐格检查相邻格子，若遇到非零方块（包括假方块），
// 则计算传感器到该格子最近边缘的距离并返回。若未遇到方块则返回传感器到边界的距离。
array<double,4> get_laser_readings_at_coords(double Xc, double Yc)
{
	array<double,4> res = {0.0,0.0,0.0,0.0};

	int center_row = (int)(Yc / CELL_SIZE) + 1; if (center_row < 1) center_row = 1; if (center_row > 4) center_row = 4;
	int center_col = (int)(Xc / CELL_SIZE) + 1; if (center_col < 1) center_col = 1; if (center_col > 3) center_col = 3;

	// front: 向上 (-Y)
	double Ys_front = Yc - HALF_ROBOT;
	bool found = false;
	for (int r = center_row - 1; r >= 1; --r) {
		int grid_c = center_col;
		if (block[r][grid_c] != 0) { double Y_bottom = r * CELL_SIZE; res[0] = max(0.0, Ys_front - Y_bottom); found = true; break; }
	}
	if (!found) res[0] = max(0.0, Ys_front - 0.0);

	// right: 向右 (+X)
	double Xs_right = Xc + HALF_ROBOT;
	found = false;
	for (int c = center_col + 1; c <= 3; ++c) {
		int grid_r = center_row;
		if (block[grid_r][c] != 0) { double X_left = (c - 1) * CELL_SIZE; res[1] = max(0.0, X_left - Xs_right); found = true; break; }
	}
	if (!found) res[1] = max(0.0, 3.0 * CELL_SIZE - Xs_right);

	// back: 向下 (+Y)
	double Ys_back = Yc + HALF_ROBOT;
	found = false;
	for (int r = center_row + 1; r <= 4; ++r) {
		int grid_c = center_col;
		if (block[r][grid_c] != 0) { double Y_top = (r - 1) * CELL_SIZE; res[2] = max(0.0, Y_top - Ys_back); found = true; break; }
	}
	if (!found) res[2] = max(0.0, 4.0 * CELL_SIZE - Ys_back);

	// left: 向左 (-X)
	double Xs_left = Xc - HALF_ROBOT;
	found = false;
	for (int c = center_col - 1; c >= 1; --c) {
		int grid_r = center_row;
		if (block[grid_r][c] != 0) { double X_right = c * CELL_SIZE; res[3] = max(0.0, Xs_left - X_right); found = true; break; }
	}
	if (!found) res[3] = max(0.0, Xs_left - 0.0);

	return res;
}

// 打印四个激光测距仪读数，格式化输出
void print_laser_readings()
{
	auto r = get_laser_readings_at_coords(posX, posY);
	// 按机器人朝向打印：front/right/back/left
	array<double,4> robot_r;
	for (int i=0;i<4;++i) robot_r[i] = r[(dir + i) % 4];
	cout << fixed << setprecision(2);
	cout << "Laser(front) : " << robot_r[0] << " m\t";
	cout << "Laser(right) : " << robot_r[1] << " m\t";
	cout << "Laser(back)  : " << robot_r[2] << " m\t";
	cout << "Laser(left)  : " << robot_r[3] << " m\n";
	cout << setprecision(6);
}

// 保持兼容：按格子中心调用的旧接口
// 辅助：朝向名称
const char* orientation_name(int d) {
	switch(d) {
		case 0: return "front(up)";
		case 1: return "right";
		case 2: return "back(down)";
		case 3: return "left";
		default: return "unknown";
	}
}

// 原地旋转
void rotate_left() {
	dir = (dir + 3) % 4;
	cout << "Rotate left -> now facing: " << orientation_name(dir) << endl;
}
void rotate_right() {
	dir = (dir + 1) % 4;
	cout << "Rotate right -> now facing: " << orientation_name(dir) << endl;
}

// 到达位置时发出信号
void emit_signal(int rx, int ry) {
	cout << "SIGNAL: reached (" << rx << "," << ry << ")" << endl;
}

// BFS solver: finds shortest path that collects all R2s and ends at (4,1) if possible; otherwise at (4,3).
void solve_and_print_path()
{
	// collect R2 coordinates
	vector<pair<int,int>> r2s;
	for (int i =1; i <=4; ++i) {
		for (int j =1; j <=3; ++j) {
			if (block[i][j] ==2) r2s.emplace_back(i,j);
		}
	}
	int R = (int)r2s.size();
	if (R ==0) {
		cout << "没有 R2，直接前往目标。" << endl;
		return;
	}
	if (R >16) R =16; // safety

	// coverage sets: for each R2, which positions collect it
	// NOTE: collection now only when robot is in one of the four adjacent cells (not the R2 cell itself)
	vector<vector<pair<int,int>>> cover(R);
	for (int k =0; k < R; ++k) {
		int rx = r2s[k].first, ry = r2s[k].second;
		const int dx[4] = {1,-1,0,0};
		const int dy[4] = {0,0,1,-1};
		for (int d =0; d <4; ++d) {
			int nx = rx + dx[d], ny = ry + dy[d];
			if (nx >=1 && nx <=4 && ny >=1 && ny <=3) cover[k].push_back({nx,ny});
		}
	}

	// helper to update mask when at position (i,j)
	auto mask_at = [&](int i, int j)->int{
		int m =0;
		for (int k =0; k < R; ++k) {
			for (auto &p : cover[k]) if (p.first==i && p.second==j) { m |= (1<<k); break; }
		}
		return m;
	};

	int fullMask = (1<<R)-1;

	// BFS state: x,y,mask
	static bool visited[5][4][1<<4]; // [1..4][1..3]
	static tuple<int,int,int> prevState[5][4][1<<4];
	static char prevMove[5][4][1<<4];
	for (int i=0;i<5;i++) for (int j=0;j<4;j++) for (int m=0;m<(1<<R);m++) visited[i][j][m]=false;

	queue<tuple<int,int,int,int>> q; // x,y,mask,depth
	int sx=1, sy=2;
	int startMask = mask_at(sx,sy);
	// don't start if starting cell is blocked by fake block
	if (block[sx][sy] ==3) {
		cout << "起始格 (1,2) 被假方块堵住，无法开始。" << endl;
		return;
	}
	visited[sx][sy][startMask]=true;
	prevState[sx][sy][startMask]=make_tuple(-1,-1,-1);
	prevMove[sx][sy][startMask]='X';
	q.emplace(sx,sy,startMask,0);

	// track best goal for each exit separately
	int bestDepth41 = -1; tuple<int,int,int> bestState41;
	int bestDepth43 = -1; tuple<int,int,int> bestState43;

	while(!q.empty()){
		auto [cx,cy,cmask,cd] = q.front(); q.pop();
		// check if this is a goal (all R2 collected)
		if (cmask == fullMask) {
			if (cx==4 && cy==1) {
				if (bestDepth41==-1 || cd < bestDepth41) { bestDepth41 = cd; bestState41 = make_tuple(cx,cy,cmask); }
			}
			if (cx==4 && cy==3) {
				if (bestDepth43==-1 || cd < bestDepth43) { bestDepth43 = cd; bestState43 = make_tuple(cx,cy,cmask); }
			}
			// even if goal found, don't stop early -- we want best for each exit
		}

		// expand neighbors
		const int dx[4] = {-1,0,1,0};
		const int dy[4] = {0,-1,0,1};
		const char mv[4] = {'W','A','S','D'}; // W: up (x-1), A: left (y-1), S: down (x+1), D: right (y+1)
		for (int d=0; d<4; ++d) {
			int nx = cx + dx[d], ny = cy + dy[d];
			if (nx <1 || nx >4 || ny <1 || ny >3) continue;
			// do not step onto fake block cells (marked3)
			if (block[nx][ny] ==3) continue;
			int nmask = cmask | mask_at(nx,ny);
			if (!visited[nx][ny][nmask]){
				visited[nx][ny][nmask]=true;
				prevState[nx][ny][nmask]=make_tuple(cx,cy,cmask);
				prevMove[nx][ny][nmask]=mv[d];
				q.emplace(nx,ny,nmask,cd+1);
			}
		}
	}

	// decide which exit to use based on new rule:
	// if fake block (3) is at (4,1), disallow exit at (4,1) entirely
	bool forbid41 = (block[4][1] ==3);

	tuple<int,int,int> goalState;
	int goalDepth = -1;
	if (forbid41) {
		if (bestDepth43 == -1) {
			cout << "由于 (4,1) 被假方块阻塞，且无法从 (4,3) 收集到所有 R2，任务失败。" << endl;
			return;
		}
		goalState = bestState43; goalDepth = bestDepth43;
	} else {
		// choose the nearer exit among those reachable
		if (bestDepth41 == -1 && bestDepth43 == -1) {
			cout << "无法找到满足条件的路径（无法收集所有 R2 或到达任一出口）。" << endl;
			return;
		} else if (bestDepth41 == -1) {
			goalState = bestState43; goalDepth = bestDepth43;
		} else if (bestDepth43 == -1) {
			goalState = bestState41; goalDepth = bestDepth41;
		} else {
			// both reachable: pick smaller depth (closer)
			if (bestDepth41 <= bestDepth43) { goalState = bestState41; goalDepth = bestDepth41; }
			else { goalState = bestState43; goalDepth = bestDepth43; }
		}
	}

	// reconstruct path from chosen goalState
	int gx = get<0>(goalState), gy = get<1>(goalState), gmask = get<2>(goalState);
	vector<pair<int,int>> path;
	vector<char> moves;
	int cx2 = gx, cy2 = gy, cm2 = gmask;
	while (true) {
		auto p = prevState[cx2][cy2][cm2];
		int px = get<0>(p), py = get<1>(p), pm = get<2>(p);
		char mv = prevMove[cx2][cy2][cm2];
		path.emplace_back(cx2,cy2);
		if (mv!='X') moves.push_back(mv);
		if (px==-1) break;
		cx2=px; cy2=py; cm2=pm;
	}
	reverse(path.begin(), path.end());
	// compute moves from path coordinates
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

	cout << "找到最短路径，步数=" << ((int)path.size()-1) << "。\n";
	cout << "目标出口： ("<<gx<<","<<gy<<")\n";
	cout << "路径坐标： ";
	for (auto &p : path) cout << "("<<p.first<<","<<p.second<<") ";
	cout << "\n";

	// 打印并仿真每一步的移动（包含原地旋转、上坡检测与到位信号）
	int curx = sx, cury = sy;
	// 初始化连续位姿（米）
	posX = (cury - 1) * CELL_SIZE + CELL_SIZE / 2.0;
	posY = (curx - 1) * CELL_SIZE + CELL_SIZE / 2.0;
	// current orientation is in global `dir` (initially 0)
	cout << "开始位置: ("<<curx<<","<<cury<<") facing "<<orientation_name(dir)<<"\n";
	for (size_t i =0; i < path.size(); ++i) {
		int nx = path[i].first, ny = path[i].second;
		cout << "目标步 " << i << ": (" << nx << "," << ny << ")\n";

		if (nx == curx && ny == cury) {
			// 到达起始点（或重复点）
			x = curx; y = cury;
			print_location();
			print_laser_readings();
			emit_signal(curx,cury);
			continue;
		}

		// 计算移动方向（期望朝向）
		int desiredDir = -1;
		if (nx == curx - 1 && ny == cury) desiredDir = 0; // up
		else if (nx == curx + 1 && ny == cury) desiredDir = 2; // down
		else if (nx == curx && ny == cury + 1) desiredDir = 1; // right
		else if (nx == curx && ny == cury - 1) desiredDir = 3; // left
		else {
			// 非邻接点（异常），直接瞬移到目标并发信号
			curx = nx; cury = ny;
			x = curx; y = cury;
			print_location(); print_laser_readings(); emit_signal(curx,cury);
			continue;
		}

		// 检查是否为上坡：比较高度
		int curHeight = box[5 - curx][cury];
		int tgtHeight = box[5 - nx][ny];
		if (tgtHeight > curHeight) {
			// 上坡：必须先面向坡（旋转到 desiredDir），并且前向激光测距为 0.03m 才能上坡
			int diff = (desiredDir - dir + 4) % 4;
			if (diff == 1) rotate_right();
			else if (diff == 2) { rotate_right(); rotate_right(); }
			else if (diff == 3) rotate_left();

			auto readings = get_laser_readings_at_coords(posX, posY);
			double frontReading = readings[dir];
			const double required = 0.03;
			const double tol = 1e-3;
			cout << "上坡检测：当前高度="<<curHeight<<" 目标高度="<<tgtHeight<<" 前向测距="<<frontReading<<" m\n";
			if (fabs(frontReading - required) > tol) {
				// 计算需要前进的连续距离（米）
				double needed = frontReading - required;
				cout << "SIGNAL: need to move forward " << fixed << setprecision(3) << needed << " m to satisfy 0.03m requirement" << endl;

				if (needed <= 0.0) {
					cout << "前向测距已小于或等于要求，无法通过前进增加距离（可能需后退或调整）。" << endl;
					return;
				}

				// 尝试以小步长（米级）前进直到满足或被阻挡（微移动可改变测距）
				const double step = 0.01; // 1cm 步长
				double moved_total = 0.0;
				bool blocked = false;
				while (frontReading > required + tol) {
					double newPosX = posX, newPosY = posY;
					if (dir == 0) newPosY = posY - step;
					else if (dir == 1) newPosX = posX + step;
					else if (dir == 2) newPosY = posY + step;
					else if (dir == 3) newPosX = posX - step;

					// 检查边界
					if (newPosX < 0.0 || newPosX > 3.0 * CELL_SIZE || newPosY < 0.0 || newPosY > 4.0 * CELL_SIZE) {
						cout << "无法继续前进：到达场地边界。" << endl; blocked = true; break;
					}

					// 检查下一位姿所在格是否被假方块阻挡
					int next_rx = (int)(newPosY / CELL_SIZE) + 1;
					int next_ry = (int)(newPosX / CELL_SIZE) + 1;
					if (next_rx < 1) next_rx = 1; if (next_rx > 4) next_rx = 4;
					if (next_ry < 1) next_ry = 1; if (next_ry > 3) next_ry = 3;
					if (block[next_rx][next_ry] == 3) { cout << "无法继续前进：前方格子被假方块阻挡。" << endl; blocked = true; break; }

					// 更新连续位姿
					posX = newPosX; posY = newPosY; moved_total += step;

					// 若进入了新的格子，则更新整数位置并发信号
					int cell_rx = (int)(posY / CELL_SIZE) + 1;
					int cell_ry = (int)(posX / CELL_SIZE) + 1;
					if (cell_rx < 1) cell_rx = 1; if (cell_rx > 4) cell_rx = 4;
					if (cell_ry < 1) cell_ry = 1; if (cell_ry > 3) cell_ry = 3;
					if (cell_rx != curx || cell_ry != cury) {
						curx = cell_rx; cury = cell_ry; x = curx; y = cury;
						print_location(); print_laser_readings(); emit_signal(curx,cury);
					}

					// 重新读取前向激光
					auto new_readings = get_laser_readings_at_coords(posX, posY);
					frontReading = new_readings[dir];
				}

				if (frontReading > required + tol) {
					double remaining = frontReading - required;
					cout << "无法通过前进完全满足，还需在格内向前移动 " << fixed << setprecision(3) << remaining << " m（或受阻）。" << endl;
					return;
				} else {
					cout << "通过微步前进共 " << fixed << setprecision(3) << moved_total << " m 后已满足前向测距要求。" << endl;
				}
			}

			// 执行上坡前进（面向目标或已通过前进达到要求）
			// 如果目标格仍为相邻且未到达（可能已经通过前进到达），则设置为目标格
			// 上坡完成后，默认将机器人置于目标格中心（连续位姿也同步更新）
			curx = nx; cury = ny;
			posX = (cury - 1) * CELL_SIZE + CELL_SIZE / 2.0;
			posY = (curx - 1) * CELL_SIZE + CELL_SIZE / 2.0;
		} else {
			// 平地或下坡：允许向任意方向移动（不强制旋转）
			curx = nx; cury = ny;
		}
		x = curx; y = cury;
		print_location(); print_laser_readings(); emit_signal(curx,cury);
	}

	// simulate to report when each R2 is collected
	vector<int> collectedAt(R, -1);
	for (int step=0; step < (int)path.size(); ++step) {
		int px = path[step].first, py = path[step].second;
		for (int k=0;k<R;++k){
			if (collectedAt[k]==-1) {
				for (auto &qpos : cover[k]) if (qpos.first==px && qpos.second==py){ collectedAt[k]=step; break; }
			}
		}
	}
	for (int k=0;k<R;++k){
		cout << "R2 #"<<k<<" at ("<<r2s[k].first<<","<<r2s[k].second<<") collected at step ";
		if (collectedAt[k]==-1) cout << "-"; else cout << collectedAt[k];
		cout << "\n";
	}
}

int main()
{
	// 使用系统 locale（通常在终端使用 UTF-8 时会正确显示中文）
	setlocale(LC_ALL, "");
	cout << "欢迎使用梅花桩控制系统！" << endl;
	cout << "初始高度在(1,2)" << endl;

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
	solve_and_print_path();

	return 0;
}
