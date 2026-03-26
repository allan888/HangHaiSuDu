using System.Windows.Media;

namespace HangHaiSudu;

public class ShipStateNode
    {
        public int K { get; }            // 当前航段索引 (已航行距离 = K * DeltaD)
        public double T { get; }         // 累计耗时 (小时)
        public double V_stw { get; }     // 当前对水航速 (Speed Through Water, 节/Knots)
        
        public double G { get; set; }    // 实际累积油耗代价 (吨)
        public double H { get; set; }    // 启发式预估剩余油耗代价 (吨)
        public double F => G + H;        // A* 综合代价

        public ShipStateNode Parent { get; } // 父节点，用于回溯路径

        public ShipStateNode(int k, double t, double v_stw, double g, ShipStateNode parent = null)
        {
            K = k;
            T = t;
            V_stw = v_stw;
            G = g;
            Parent = parent;
        }
    }
public class VisualNode
{
    public double X { get; set; }
    public double Y { get; set; }
    public Brush Color { get; set; }
    public string ToolTip { get; set; }
    public int ZIndex { get; set; } = 1;
}

public class VisualEdge
{
    public double X1 { get; set; }
    public double Y1 { get; set; }
    public double X2 { get; set; }
    public double Y2 { get; set; }
    public Brush Color { get; set; }
    public double Thickness { get; set; } = 1;
    public int ZIndex { get; set; } = 0;
}

public class VisualEnv
{
    public double X { get; set; }
    public double Width { get; set; }
    public Brush Background { get; set; }
    public string Text { get; set; }
}
    /// <summary>
    /// 环境与启发式代价计算器
    /// </summary>
    public class HeuristicCalculator
    {
        private readonly double _totalDistance;  // 总航程 (海里)
        private readonly double _deltaD;         // 单个航段长度 (海里)
        private readonly double _tMax;           // 最大允许耗时 (小时)
        private readonly double _fuelCoeff;      // 燃油效率系数 (简化的三次曲线系数)
        
        // 船舶基础属性
        public double MaxSpeed { get; } = 15.0;  // 最大对水航速
        public double MinSpeed { get; } = 3.0;   // 最小对水航速 (维持舵效)

        public HeuristicCalculator(double totalDistance, double deltaD, double tMax, double fuelCoeff = 0.002)
        {
            _totalDistance = totalDistance;
            _deltaD = deltaD;
            _tMax = tMax;
            _fuelCoeff = fuelCoeff;
        }

        /// <summary>
        /// 数学函数 1: 计算静水单位时间油耗率 F(V_stw)
        /// 采用典型的船舶功率与速度的三次方关系
        /// </summary>
        public double GetFuelRate(double v_stw)
        {
            if (v_stw <= 0) return 0;
            // 油耗 = k * v^3 (吨/小时)
            return _fuelCoeff * Math.Pow(v_stw, 3);
        }

        /// <summary>
        /// 数学函数 2: 状态转移，计算前往下一个节点产生的时间和实际代价 g(S)
        /// </summary>
        public (double nextT, double stepCost) CalculateTransition(ShipStateNode current, double next_v_stw, double current_v_c)
        {
            double v_sog = next_v_stw + current_v_c; // 对地航速 = 对水航速 + 流速
            
            // 剪枝条件：如果遇到强逆流导致对地速度倒退或停滞
            if (v_sog <= 0.1) 
                return (double.PositiveInfinity, double.PositiveInfinity);

            double timeStep = _deltaD / v_sog;
            double nextT = current.T + timeStep;
            
            // 该航段实际油耗 = 单位时间油耗 * 耗时
            double stepCost = GetFuelRate(next_v_stw) * timeStep;

            return (nextT, stepCost);
        }

        /// <summary>
        /// 数学函数 3: 计算启发式函数 h(S)
        /// 综合了绝对经济航速下界 (H1) 和时间紧迫下界 (H2)
        /// </summary>
        public double CalculateHeuristic(ShipStateNode state, double max_future_vc)
        {
            double dRem = _totalDistance - (state.K * _deltaD); // 剩余距离
            if (dRem <= 0) return 0; // 已到达终点

            double tRem = _tMax - state.T; // 剩余可用时间
            if (tRem <= 0) return double.PositiveInfinity; // 已经超时，死路

            // --- H1: 理想经济航速下界 (假设时间无限，全程最大顺流) ---
            // 理论上，(k * v^3) / (v + vc) 取得最小值时的 v_stw 即为理想经济对水航速
            // 为简化计算，在实际工程中常通过一维搜索或导数求根获得 v_eco_ideal
            // 这里我们用微积分求导结论: 2*v^3 - 3*vc*v^2 = 0 简化推导最优 v_eco
            double v_eco_ideal = Math.Max(MinSpeed, 1.5 * max_future_vc); 
            if (v_eco_ideal > MaxSpeed) v_eco_ideal = MaxSpeed;
            
            double best_sog_h1 = v_eco_ideal + max_future_vc;
            double h1 = dRem * (GetFuelRate(v_eco_ideal) / best_sog_h1);

            // --- H2: 时间约束逼迫的下界 (全程最大顺流，但必须按时到达) ---
            double required_sog = dRem / tRem; // 必须达到的最低平均对地速度
            double required_v_stw = required_sog - max_future_vc; // 倒推需要的对水速度
            
            double h2 = 0;
            if (required_v_stw > MaxSpeed)
            {
                h2 = double.PositiveInfinity; // 即便在最大顺流下，全速也无法按时到达，无解
            }
            else if (required_v_stw <= MinSpeed)
            {
                // 时间非常宽裕，H2 退化，油耗将由 H1 主导
                h2 = 0; 
            }
            else
            {
                // 必须以 required_v_stw 航行才能恰好准时到达
                h2 = dRem * (GetFuelRate(required_v_stw) / required_sog);
            }

            // 返回最紧凑的可采纳下界 (Admissible Heuristic)
            return Math.Max(h1, h2);
        }
    }