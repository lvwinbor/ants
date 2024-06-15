#include "cubic_spline.h"

#include <utility>

CubicSplineClass::CubicSplineClass(ros::Publisher trajectory_pub) {
    _trajectory_pub = trajectory_pub;
    keeper.resize(3);
    formation_diamond.resize(5);
    formation_triangle.resize(5);
}

CSRefPath &CSRefPath::operator=(const CSRefPath &input) {
    this->CSclear();
    this->r_path = input.r_path;
    return *this;
}

void CSRefPath::joint(const CSRefPath &path) {
    this->copypath(path, 0);
    std::cout << "[cs_planner]: pre_joint_s = " << r_path.back().r_s << std::endl;
    std::cout << "[cs_planner]: new_seg_s = " << path.r_path.back().r_s << std::endl;
    std::cout << "[cs_planner]: post_joint_s = " << r_path.back().r_s << std::endl;
}

void CSRefPath::copypath(const CSRefPath &path, int index) {
    double original_length = r_path.back().r_s;
    double initial_length = path.r_path[index].r_s;
    auto insert_size = path.r_path.size() - 1 - index;// 插入数据的数量
    auto capacity = r_path.size() + insert_size;      // 应该分配的容量，第一个数据为当前位置，所以减1

    r_path.reserve(capacity);

    r_path.insert(r_path.end(), path.r_path.begin() + index + 1, path.r_path.end());// 将新数据插入到原数据后面)

    // 弧长是根据做差算出来的，只是一组的弧长数据，所以需要减去提取数据中第一个弧长值并加上原数据弧长累计值
    for (auto i = capacity - insert_size; i < capacity; ++i) {
        r_path[i].r_s -= initial_length;
        r_path[i].r_s += original_length;
    }

    std::cout << "[cs_planner]: match_point_s = " << initial_length << "[" << index << "]" << std::endl;
    std::cout << "[cs_planner]: last_total_length = " << path.r_path.back().r_s << ",now_total_length = " << r_path.back().r_s << std::endl;
}

void CSRefPath::CSclear() {
    // 使用swap可以清空内存而不只是清除数据，更加彻底
    std::vector<RefPath>().swap(r_path);// 清空内存
}

void CSRefPath::load(Spline2D &spline, double overall_length) {
    double refspline_length = spline.path.back().r_s;                       // 获得总弧长
    double step = overall_length / 8;                                       // 不懂
    int num_elements = static_cast<int>(std::ceil(refspline_length / step));// 计算需要多少步长
    std::vector<double> s;
    s.reserve(num_elements + 1);
    for (int i = 0; i < num_elements; ++i) {
        s.emplace_back(i * step);
    }
    s.emplace_back(refspline_length);
    spline.interpolate(s);
    r_path = spline.path;
    std::cout << "[cs_planner]: CSRefPath_size=" << r_path.size() << std::endl;
}


std::vector<double> vec_diff(const std::vector<double> &input) {
    std::vector<double> output;
    output.reserve(input.size());// output容器要有足够的容量，否则报越界错误
    std::adjacent_difference(input.begin(), input.end(), output.begin());
    output.erase(output.begin());//第一个元素是原始数据
    return output;
}

std::vector<double> cum_sum(const std::vector<double> &input) {
    std::vector<double> output;
    output.reserve(input.size());
    std::partial_sum(input.begin(), input.end(), output.begin());
    return output;
}

double Spline::calc(double t) {
    if (t < point.front().x || t > point.back().x) {
        throw std::invalid_argument("received value out of the pre-defined range");
    }
    int seg_id = bisect(t, 0, nx);
    double dx = t - point[seg_id].x;
    auto p = parameter[seg_id];
    return p.a + p.b * dx + p.c * dx * dx + p.d * dx * dx * dx;
};

double Spline::calc_d(double t) {
    if (t < point.front().x || t > point.back().x) {
        throw std::invalid_argument("received value out of the pre-defined range");
    }
    int seg_id = bisect(t, 0, nx - 1);
    double dx = t - point[seg_id].x;
    auto p = parameter[seg_id];
    return p.b + 2 * p.c * dx + 3 * p.d * dx * dx;
}

double Spline::calc_dd(double t) {
    if (t < point.front().x || t > point.back().x) {
        throw std::invalid_argument("received value out of the pre-defined range");
    }
    int seg_id = bisect(t, 0, nx);
    double dx = t - point[seg_id].x;
    auto p = parameter[seg_id];
    return 2 * p.c + 6 * p.d * dx;
}

Eigen::MatrixXd Spline::calc_A() {
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(nx + 1, nx + 1);
    /*自由边界，即第一段第一个点和最后一点最后一个点的二阶导数为0
    */
    for (int i = 1; i < A.rows() - 1; ++i) {
        A(i, i - 1) = h[i - 1];
        A(i, i + 1) = h[i];
        A(i, i) = 2 * (A(i, i - 1) + A(i, i + 1));
    }
    A(0, 0) = 2;
    A(nx - 1, nx - 1) = 2;
    return A;
    /*受约束边界，即第一段第一个点和最后一点最后一个点的一阶导数为给定值
    
    */
}

Eigen::VectorXd Spline::calc_B() {
    Eigen::VectorXd B = Eigen::VectorXd::Zero(nx);
    // 创建vector y便于公式书写
    std::vector<double> y;
    y.reserve(point.size());
    for (const auto &i: point) {
        y.emplace_back(i.y);
    }
    for (int i = 1; i < B.size() - 1; i++)
        B(i) = (y[i + 1] - y[i]) / h[i] - (y[i] - y[i - 1]) / h[i - 1];
    return B;
}

int Spline::bisect(double t, int start, int end) {
    while (end - start > 1) {
        int mid = (start + end) / 2;
        if (t == point[mid].x) {
            return mid;
        } else if (t > point[mid].x) {
            start = mid;
        } else {
            end = mid;
        }
    }
    return start;// 当 end - start <= 1 时，返回 start
}

Spline::Spline(const std::vector<double> &x_, const std::vector<double> &y_) {
    // 因为拟合曲线输入的两组数据无法确定关系，无法使用结构体约束x与y的数量，需要加判断
    if (x_.size() != y_.size()) {
        std::cerr << "三次插值时x与y的数量不相等!" << std::endl;
        throw std::invalid_argument("三次插值时x与y的数量不相等");
    }

    nx = static_cast<int>(x_.size() - 1);
    h = vec_diff(x_);

    parameter.reserve(nx);
    point.reserve(nx + 1);

    //推导可见https://zhuanlan.zhihu.com/p/672601034
    // 构造以系数c为未知量的矩阵方程并求解，公式易知，直接可输入此矩阵方程的系数矩阵A和解向量B
    Eigen::MatrixXd A;
    Eigen::VectorXd B, m_temp;
    A = calc_A();
    B = calc_B();
    m_temp = A.colPivHouseholderQr().solve(6 * B);
    // 存储顺序默认使用列存储，可以使用data()函数获得存储数据的首地址
    /* Map类用于通过C++中普通的连续指针或者数组 （raw
  C/C++arrays）来构造Eigen里的Matrix类，
   这就好比Eigen里的Matrix类的数据和raw
  C++array共享了一片地址，也就是引用。
   比如有个API只接受普通的C++数组，但又要对普通数组进行线性代数操作，那么用它构造为Map类，
   直接操作Map就等于操作了原始普通数组，省时省力。
   STL中不同容器之间是不能直接赋值的，assign（）可以实现不同容器但相容的类型赋值*/
    std::vector<double> m(m_temp.data(), m_temp.data() + m_temp.size());
    // 创建vector y便于公式书写
    std::vector<double> y;
    y.reserve(point.size());
    for (const auto &i: point) {
        y.emplace_back(i.y);
    }
    // 套公式求出系数b和d
    for (int i = 0; i < nx; ++i) {
        point[i].x = x_[i];
        point[i].y = y_[i];

        parameter[i].a = y[i];
        parameter[i].c = m[i];
        parameter[i].d = (m[i + 1] - m[i]) / (6 * h[i]);
        parameter[i].d = ((y[i + 1] - y[i]) / h[i] - h[i] * m[i] / 2 - (m[i + 1] - m[i]) * h[i] / 6);
    }
    point[nx].x = x_[nx];
    point[nx].y = y_[nx];
}


Point Spline2D::calc_position(double s_t) {
    Point point{};
    point.x = sx.calc(s_t);
    point.y = sy.calc(s_t);
    return point;
};

double Spline2D::calc_curvature(double s_t) {
    double dx = sx.calc_d(s_t);
    double ddx = sx.calc_dd(s_t);
    double dy = sy.calc_d(s_t);
    double ddy = sy.calc_dd(s_t);

    // 曲率计算公式
    return (ddy * dx - ddx * dy) / pow((dx * dx + dy * dy), 1.5);
};

double Spline2D::calc_yaw(double s_t) {
    double dx = sx.calc_d(s_t);
    double dy = sy.calc_d(s_t);
    return std::atan2(dy, dx);
};

std::vector<double> Spline2D::calc_s(const std::vector<double> &x, const std::vector<double> &y) {
    std::vector<double> ds;
    ds.reserve(x.size());
    std::vector<double> dx = vec_diff(x);
    std::vector<double> dy = vec_diff(y);
    std::transform(dx.begin(), dx.end(), dy.begin(), ds.begin(),
                   [](double x, double y) { return std::sqrt(x * x + y * y); });
    ds.emplace(ds.begin(), 0);// ds的数量需要与x一致
    return cum_sum(ds);
}
Spline2D::Spline2D(const std::vector<Point> &point) {

    std::vector<double> x_values;
    std::vector<double> y_values;
    x_values.reserve(point.size());
    y_values.reserve(point.size());
    for (const auto i: point) {
        x_values.emplace_back(i.x);
        y_values.emplace_back(i.y);
    }

    s = calc_s(x_values, y_values);
    sx = Spline(s, x_values);
    sy = Spline(s, y_values);
    interpolate(s);
}
void Spline2D::interpolate(const std::vector<double> &s_vector) {
    path.reserve(s_vector.size());
    path.resize(s_vector.size());
    for (int i = 0; i < s_vector.size(); ++i) {
        auto point = calc_position(s_vector[i]);
        path[i].r_x = point.x;
        path[i].r_y = point.y;
        path[i].r_curvature = calc_curvature(s_vector[i]);
        path[i].r_yaw = calc_yaw(s_vector[i]);
        path[i].r_s = s_vector[i];
    }
}


void CubicSplineClass::UpdateKeeper(common_private_msgs::vehicle_status last_back) {
    keeper[0].x = mystate.x;
    keeper[0].y = mystate.y;
    keeper[0].yaw = mystate.yaw;
    keeper[1].x = last_back.xPos;
    keeper[1].y = last_back.yPos;
    keeper[1].yaw = last_back.yaw;
}

int CubicSplineClass::findMatchpoint(double x, double y, CSRefPath &input_trajectory) {
    int num = input_trajectory.r_path.size();
    double dis_min = std::numeric_limits<double>::max();
    int index = 0;
    for (int i = 0; i < num; i++) {
        double temp_dis = std::pow(input_trajectory.r_path[i].r_x - x, 2) + std::pow(input_trajectory.r_path[i].r_y - y, 2);
        //ROS_ERROR("%s,%f.%s,%f","r_x",r_x[i],"r_y",r_y[i]);
        //ROS_ERROR("%s,%f","distance",temp_dis);
        if (temp_dis < dis_min) {
            dis_min = temp_dis;
            index = i;
        }
    }
    // ROS_ERROR("%s,%i","nearest point",index);
    return index;
}
