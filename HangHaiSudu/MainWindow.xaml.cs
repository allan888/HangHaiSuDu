using System.Collections.ObjectModel;
using System.Text;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;

namespace HangHaiSudu;

/// <summary>
/// Interaction logic for MainWindow.xaml
/// </summary>
public partial class MainWindow : Window
    {
        public ObservableCollection<VisualNode> Nodes { get; set; } = new ObservableCollection<VisualNode>();
        public ObservableCollection<VisualEdge> Edges { get; set; } = new ObservableCollection<VisualEdge>();
        public ObservableCollection<VisualEnv> Environments { get; set; } = new ObservableCollection<VisualEnv>();
        
        private double SegmentPixelWidth = 80; 
        private const double TimePixelScale = 45; // 纵向：1小时 = 45像素

        
        private double _totalDistance;
        private int _totalSegments;
        private double _deltaD;
        private double _tMax;
        private double[] _oceanCurrents;
        private double[] _availableSpeeds;

        public MainWindow()
        {
            InitializeComponent();
            DataContext = this;
            ApplyParameters();
        }
        private void BtnApply_Click(object sender, RoutedEventArgs e)
        {
            ApplyParameters();
        }

        private void ApplyParameters()
        {
            try
            {
                
                _totalDistance = double.Parse(TxtDistance.Text);
                _totalSegments = int.Parse(TxtSegments.Text);
                _tMax = double.Parse(TxtTMax.Text);
                
                _availableSpeeds = TxtSpeeds.Text.Split(',')
                    .Select(s => double.Parse(s.Trim())).ToArray();
                
                _oceanCurrents = TxtCurrents.Text.Split(',')
                    .Select(s => double.Parse(s.Trim())).ToArray();
                
                if (_oceanCurrents.Length != _totalSegments)
                {
                    MessageBox.Show($"参数冲突！你设定了 {_totalSegments} 个航段，但输入了 {_oceanCurrents.Length} 个洋流数据。请保持一致。", "参数错误", MessageBoxButton.OK, MessageBoxImage.Warning);
                    return;
                }

                
                _deltaD = _totalDistance / _totalSegments;
                
                MainCanvas.Width = Math.Max(1000, _totalSegments * 90); 
                SegmentPixelWidth = MainCanvas.Width / _totalSegments;
                
                Nodes.Clear();
                Edges.Clear();
                
                DrawEnvironment();
                
                TxtStatus.Text = "参数已更新，等待开始寻路...";
                TxtStatus.Foreground = Brushes.LightGray;
            }
            catch (Exception ex)
            {
                MessageBox.Show($"参数格式有误，请确保输入的是数字，并使用英文逗号分隔。\n详细错误: {ex.Message}", "解析错误", MessageBoxButton.OK, MessageBoxImage.Error);
            }
        }
        
        private void DrawEnvironment()
        {
            Environments.Clear();

            for (int i = 0; i < _totalSegments; i++)
            {
                double current = _oceanCurrents[i];
                Brush bg = new SolidColorBrush(Color.FromArgb(30, 128, 128, 128)); // 静水灰
                if (current > 0) bg = new SolidColorBrush(Color.FromArgb(40, 0, 255, 0)); // 顺流绿
                if (current < 0) bg = new SolidColorBrush(Color.FromArgb(40, 255, 0, 0)); // 逆流红

                Environments.Add(new VisualEnv
                {
                    X = i * SegmentPixelWidth,
                    Width = SegmentPixelWidth,
                    Background = bg,
                    Text = current == 0 ? "静水" : $"流速\n{current}节"
                });
            }
            
            double tMaxY = _tMax * TimePixelScale;
            LineTMax.Y1 = tMaxY;
            LineTMax.Y2 = tMaxY;
            Canvas.SetTop(TxtTMaxLabel, tMaxY + 5);
        }
        
        private async void BtnStart_Click(object sender, RoutedEventArgs e)
        {
            ApplyParameters();

            Nodes.Clear();
            Edges.Clear();
            TxtStatus.Text = "正在计算状态空间树，请稍候...";
            TxtStatus.Foreground = Brushes.Yellow;
            
            BtnStart.IsEnabled = false;
            
            await RunAStarAnimation();
            
            BtnStart.IsEnabled = true;
        }

        private async Task RunAStarAnimation()
        {
            var calculator = new HeuristicCalculator(_totalDistance, _deltaD, _tMax);
            var openList = new PriorityQueue<ShipStateNode, double>();
            var closedList = new Dictionary<string, double>();

            // 假设默认初始速度为可用速度的中间值
            double initialSpeed = _availableSpeeds[_availableSpeeds.Length / 2];
            var startNode = new ShipStateNode(0, 0, initialSpeed, 0, null);
            startNode.H = calculator.CalculateHeuristic(startNode, _oceanCurrents.Max());
            openList.Enqueue(startNode, startNode.F);
            closedList[$"0_{initialSpeed}"] = 0;

            ShipStateNode goalNode = null;
            int expandedNodes = 0; // 记录探索了多少个节点

            while (openList.Count > 0)
            {
                var current = openList.Dequeue();
                expandedNodes++;

                // 画出当前探索的节点
                AddVisualNode(current, Brushes.LimeGreen);

                if (current.K == _totalSegments)
                {
                    if (current.T <= _tMax)
                    {
                        goalNode = current;
                        break; // 成功！
                    }
                    continue; 
                }

                double currentV_C = _oceanCurrents[current.K];
                double maxFutureC = _oceanCurrents.Skip(current.K + 1).DefaultIfEmpty(0).Max();

                foreach (var nextSpeed in _availableSpeeds)
                {
                    var (nextT, stepCost) = calculator.CalculateTransition(current, nextSpeed, currentV_C);
                    if (double.IsInfinity(stepCost) || nextT > _tMax) continue;

                    double nextG = current.G + stepCost;
                    string stateKey = $"{current.K + 1}_{nextSpeed}";
                    
                    if (closedList.TryGetValue(stateKey, out double bestG) && bestG <= nextG)
                        continue; 

                    var nextNode = new ShipStateNode(current.K + 1, nextT, nextSpeed, nextG, current);
                    nextNode.H = calculator.CalculateHeuristic(nextNode, maxFutureC);

                    if (double.IsInfinity(nextNode.H)) continue;

                    closedList[stateKey] = nextG;
                    openList.Enqueue(nextNode, nextNode.F);

                    AddVisualEdge(current, nextNode, Brushes.DimGray, 1, 0);
                }
                
                await Task.Delay(15); 
            }

            if (goalNode != null)
            {
                TxtStatus.Text = $"规划成功！探索节点数: {expandedNodes}\n最优总油耗: {goalNode.G:F3} 吨\n实际总耗时: {goalNode.T:F2} 小时";
                TxtStatus.Foreground = Brushes.LimeGreen;
                HighlightOptimalPath(goalNode);
            }
            else
            {
                TxtStatus.Text = $"搜索失败。探索了 {expandedNodes} 个节点后，发现在当前环境和允许速度下，无论如何都无法在 {_tMax} 小时内到达终点。";
                TxtStatus.Foreground = Brushes.Red;
            }
        }

        private void AddVisualNode(ShipStateNode node, Brush color)
        {
            Nodes.Add(new VisualNode
            {
                X = node.K * SegmentPixelWidth,
                Y = node.T * TimePixelScale,
                Color = color,
                ToolTip = $"航段: {node.K}\n耗费时间: {node.T:F2}h\n对水航速: {node.V_stw}节\n当前耗油: {node.G:F3}吨",
                ZIndex = 2
            });
        }

        private void AddVisualEdge(ShipStateNode parent, ShipStateNode child, Brush color, double thickness, int zIndex)
        {
            Edges.Add(new VisualEdge
            {
                X1 = parent.K * SegmentPixelWidth,
                Y1 = parent.T * TimePixelScale,
                X2 = child.K * SegmentPixelWidth,
                Y2 = child.T * TimePixelScale,
                Color = color,
                Thickness = thickness,
                ZIndex = zIndex
            });
        }

        private async void HighlightOptimalPath(ShipStateNode goalNode)
        {
            var path = new List<ShipStateNode>();
            var curr = goalNode;
            while (curr != null)
            {
                path.Add(curr);
                curr = curr.Parent;
            }
            path.Reverse();

            // 1. 播放高亮连线动画
            for (int i = 0; i < path.Count - 1; i++)
            {
                AddVisualEdge(path[i], path[i + 1], Brushes.OrangeRed, 4, 10);
                AddVisualNode(path[i + 1], Brushes.OrangeRed);
                await Task.Delay(40);
            }

            // 2. 收集路径数据用于报表展示
            var reportData = new List<PathDetailRecord>();
            for (int i = 0; i < path.Count; i++)
            {
                var node = path[i];
                
                // 起点没有遭遇流速和对地航速
                string currentStr = "-";
                string sogStr = "-";
                
                if (i > 0)
                {
                    double vc = _oceanCurrents[node.K - 1]; // 到达该节点所经历的那个航段的流速
                    double sog = node.V_stw + vc;
                    currentStr = $"{vc:F1} 节";
                    sogStr = $"{sog:F1} 节";
                }

                reportData.Add(new PathDetailRecord
                {
                    航段节点 = i == 0 ? "起点 (K=0)" : $"第 {node.K} 段",
                    遭遇流速 = currentStr,
                    指令对水航速 = $"{node.V_stw:F1} 节",
                    实际对地航速 = sogStr,
                    节点累计耗时 = $"{node.T:F2} 小时",
                    节点累计油耗 = $"{node.G:F3} 吨"
                });
            }

            // 3. 动态生成并弹出一个 WPF 窗口来展示表格
            ShowResultWindow(reportData);
        }

        // 动态生成弹窗的辅助方法
        private void ShowResultWindow(List<PathDetailRecord> data)
        {
            // 创建数据表格
            var dataGrid = new DataGrid
            {
                ItemsSource = data,
                AutoGenerateColumns = true,
                IsReadOnly = true,
                AlternatingRowBackground = new SolidColorBrush(Color.FromArgb(255, 240, 240, 240)),
                HeadersVisibility = DataGridHeadersVisibility.Column,
                GridLinesVisibility = DataGridGridLinesVisibility.All,
                FontSize = 14,
                Margin = new Thickness(10)
            };

            // 创建弹窗
            var resultWindow = new Window
            {
                Title = "航线规划详细数据报表",
                Width = 700,
                Height = 400,
                WindowStartupLocation = WindowStartupLocation.CenterOwner,
                Owner = this, // 设为主窗口的子窗口
                Background = Brushes.White,
                Content = dataGrid
            };

            // 显示弹窗
            resultWindow.ShowDialog();
        }
    }
public class PathDetailRecord
{
    public string 航段节点 { get; set; }
    public string 遭遇流速 { get; set; }
    public string 指令对水航速 { get; set; }
    public string 实际对地航速 { get; set; }
    public string 节点累计耗时 { get; set; }
    public string 节点累计油耗 { get; set; }
}