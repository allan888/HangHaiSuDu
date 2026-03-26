namespace HangHaiSudu;

public class QiFa
    {
        private readonly HeuristicCalculator _calculator;
        private readonly int _totalSegments;
        private readonly double[] _oceanCurrents; // 环境地图：每个航段的顺/逆流速度
        private readonly double[] _availableSpeeds; // 离散化的动作空间 (可选对水航速)
        private readonly double _tMax;

        public QiFa(HeuristicCalculator calculator, int totalSegments, double[] oceanCurrents, double[] availableSpeeds, double tMax)
        {
            _calculator = calculator;
            _totalSegments = totalSegments;
            _oceanCurrents = oceanCurrents;
            _availableSpeeds = availableSpeeds;
            _tMax = tMax;
        }

        /// <summary>
        /// A* 搜索主程序
        /// </summary>
        public ShipStateNode FindOptimalSpeedProfile(double initialSpeed)
        {
            // Open List：按照节点总代价 F = G + H 从小到大排序
            var openList = new PriorityQueue<ShipStateNode, double>();
            
            // Closed List：记录已访问状态。为了简化，我们以 "航段K_对水航速V" 作为状态键，记录到达该状态的最小实际油耗 G
            var closedList = new Dictionary<string, double>();

            // 1. 初始化起点
            var startNode = new ShipStateNode(0, 0, initialSpeed, 0, null);
            startNode.H = _calculator.CalculateHeuristic(startNode, GetMaxFutureCurrent(0));
            
            openList.Enqueue(startNode, startNode.F);
            closedList[GetStateKey(startNode)] = startNode.G;

            // 2. 开始 A* 循环
            while (openList.Count > 0)
            {
                // 取出当前 F 值最小的节点
                var currentNode = openList.Dequeue();

                // 目标检查：是否到达终点且未超时
                if (currentNode.K == _totalSegments)
                {
                    if (currentNode.T <= _tMax)
                    {
                        Console.WriteLine("🎉 搜索成功！找到全局最优解。");
                        return currentNode; // 找到最优解，返回终点节点以供回溯
                    }
                    continue; // 虽然到了，但是超时了，这条路作废
                }

                // 获取当前航段的洋流环境
                double currentOceanCurrent = _oceanCurrents[currentNode.K];
                // 获取未来可能的最大顺流 (用于计算 H)
                double maxFutureCurrent = GetMaxFutureCurrent(currentNode.K + 1);

                // 3. 扩展子节点 (遍历所有可选的对水航速)
                foreach (var nextSpeed in _availableSpeeds)
                {
                    // 计算状态转移 (时间增量和油耗增量)
                    var (nextT, stepCost) = _calculator.CalculateTransition(currentNode, nextSpeed, currentOceanCurrent);

                    // 剪枝 1：如果遇到极端逆流导致停滞，或者总耗时已经超过了 TMax，直接抛弃
                    if (double.IsInfinity(stepCost) || nextT > _tMax)
                        continue;

                    // 生成新状态
                    double nextG = currentNode.G + stepCost;
                    var nextNode = new ShipStateNode(currentNode.K + 1, nextT, nextSpeed, nextG, currentNode);
                    
                    // 剪枝 2：Closed List 检查 (核心防爆栈逻辑)
                    string stateKey = GetStateKey(nextNode);
                    if (closedList.TryGetValue(stateKey, out double bestG))
                    {
                        // 如果之前到达过相同的 (位置, 速度) 状态，且之前的油耗更低，则跳过当前分支
                        // (注意：严谨的带时间窗的路径规划中，还需要对比时间 T。这里做简化，假设 H 函数能处理好时间约束)
                        if (bestG <= nextG) continue;
                    }

                    // 计算启发式代价 H
                    nextNode.H = _calculator.CalculateHeuristic(nextNode, maxFutureCurrent);

                    // 剪枝 3：如果 H 为无穷大，说明从这里开始就算一直全速且最大顺流也会超时
                    if (double.IsInfinity(nextNode.H))
                        continue;

                    // 记录到 Closed List 并加入 Open List
                    closedList[stateKey] = nextG;
                    openList.Enqueue(nextNode, nextNode.F);
                }
            }

            Console.WriteLine("❌ 搜索失败：在规定时间内无法到达终点。");
            return null; // 队列空了也没找到可行解
        }

        // 辅助方法：生成状态键
        private string GetStateKey(ShipStateNode node) => $"{node.K}_{node.V_stw}";

        // 辅助方法：获取前方航程的最大顺流
        private double GetMaxFutureCurrent(int startK)
        {
            double maxC = 0;
            for (int i = startK; i < _totalSegments; i++)
            {
                if (_oceanCurrents[i] > maxC) maxC = _oceanCurrents[i];
            }
            return maxC;
        }
    }