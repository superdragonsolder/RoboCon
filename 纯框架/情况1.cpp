#include<iostream>
#include<random>
#include<vector>
#include<algorithm>
#include<queue>
#include<climits>
#include<tuple>
#include<array>
#include<locale>

using namespace std;

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

	// 打印每一步的路径（逐步）
	for (size_t i =0; i < path.size(); ++i) {
		cout << "步 " << i << ": (" << path[i].first << "," << path[i].second << ")\n";
		// 设置全局位置并打印梅花桩示意图
		x = path[i].first;
		y = path[i].second;
		print_location();
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
	// 设置区域为系统默认，支持中文输出
	setlocale(LC_ALL, "");
	cout.imbue(locale(""));
	
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
