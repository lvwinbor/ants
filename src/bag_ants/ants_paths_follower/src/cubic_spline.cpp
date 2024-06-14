#include "cubic_spline.h"

#include <utility>

CubicSplineClass::CubicSplineClass(ros::Publisher trajectory_pub) {
    _trajectory_pub = trajectory_pub;
    keeper.resize(3);
    formation_diamond.resize(5);
    formation_triangle.resize(5);
}

CSRefPath &CSRefPath::operator=(const CSRefPath &input) {
    std::vector<double>().swap(r_curvature);// 清空内存
    std::vector<double>().swap(r_s);
    std::vector<double>().swap(r_x);
    std::vector<double>().swap(r_y);
    std::vector<double>().swap(r_yaw);
    this->r_curvature = input.r_curvature;
    this->r_s = input.r_s;
    this->r_x = input.r_x;
    this->r_y = input.r_y;
    this->r_yaw = input.r_yaw;
    return *this;
}

void CSRefPath::joint(const CSRefPath &path) {
    this->copypath(path, 0);
    std::cout << "[cs_planner]: pre_joint_s = " << r_s.back() << std::endl;
    std::cout << "[cs_planner]: new_seg_s = " << path.r_s.back() << std::endl;
    std::cout << "[cs_planner]: post_joint_s = " << r_s.back() << std::endl;
}

void CSRefPath::copypath(const CSRefPath &path, int index) {
    double original_length = r_s.back();
    double initial_length = path.r_s[index];
    int insert_size = static_cast<int>(path.r_s.size() - 1 - index);// 插入数据的数量
    int capacity = static_cast<int>(r_s.size()) + insert_size;      // 应该分配的容量，第一个数据为当前位置，所以减1

    r_curvature.reserve(capacity);
    r_x.reserve(capacity);
    r_y.reserve(capacity);
    r_yaw.reserve(capacity);
    r_s.reserve(capacity);

    r_curvature.insert(r_curvature.end(), path.r_curvature.begin() + index + 1, path.r_curvature.end());// 将新数据插入到原数据后面
    r_x.insert(r_x.end(), path.r_x.begin() + index + 1, path.r_x.end());
    r_y.insert(r_y.end(), path.r_y.begin() + index + 1, path.r_y.end());
    r_yaw.insert(r_yaw.end(), path.r_yaw.begin() + index + 1, path.r_yaw.end());
    std::vector<double> diff_r_s;
    diff_r_s.reserve(insert_size);
    std::transform(path.r_s.end() - insert_size, path.r_s.end(), diff_r_s.begin(), [initial_length](int n) { return n - initial_length; });// 弧长是根据做差算出来的，所以提取时要减去第一个弧长
    r_s.insert(r_s.end(), diff_r_s.begin(), diff_r_s.end());
    auto new_data_ptr = r_s.end() - insert_size;
    std::transform(new_data_ptr, r_s.end(), new_data_ptr, [original_length](int n) { return n + original_length; });// 弧长是根据做差算出来的，只是一组的弧长数据，所以需要加上原数据弧长累计值
    std::cout << "[cs_planner]: match_point_s = " << initial_length << "[" << index << "]" << std::endl;
    std::cout << "[cs_planner]: last_total_length = " << path.r_s.back() << ",now_total_length = " << r_s.back() << std::endl;
}

void CSRefPath::CSclear() {
    r_curvature.clear();
    r_s.clear();
    r_x.clear();
    r_y.clear();
    r_yaw.clear();
}

void CSRefPath::load(Spline2D &spline, double overall_length) {
    double refspline_length = spline.length();                              // 获得总弧长
    double step = overall_length / 8;                                       // 不懂
    int num_elements = static_cast<int>(std::ceil(refspline_length / step));// 计算需要多少步长
    r_s.reserve(num_elements + 1);
    std::generate(r_s.begin(), r_s.begin() + num_elements, [n = 0.0, step]() mutable {
        double current = n;
        n += step;
        return current;
    });
    r_s.emplace_back(refspline_length);
    r_x.reserve(r_s.size());
    r_y.reserve(r_s.size());
    r_yaw.reserve(r_s.size());
    r_curvature.reserve(r_s.size());
    for (const auto i: r_s) {
        Point point_ = spline.calc_position(i);// 计算弧长为i时的坐标
        r_x.emplace_back(point_.x);
        r_y.emplace_back(point_.y);
        r_yaw.emplace_back(spline.calc_yaw(i));            // 计算弧长为i时的航向角
        r_curvature.emplace_back(spline.calc_curvature(i));// 计算弧长为i时的曲率
    }
    std::cout << "[cs_planner]: CSRefPath_size=" << r_curvature.size() << std::endl;
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
    if (t < x.front() || t > x.back()) {
        throw std::invalid_argument("received value out of the pre-defined range");
    }
    int seg_id = bisect(t, 0, nx);
    double dx = t - x[seg_id];
    return a[seg_id] + b[seg_id] * dx + c[seg_id] * dx * dx + d[seg_id] * dx * dx * dx;
};

double Spline::calc_d(double t) {
    if (t < x.front() || t > x.back()) {
        throw std::invalid_argument("received value out of the pre-defined range");
    }
    int seg_id = bisect(t, 0, nx - 1);
    double dx = t - x[seg_id];
    return b[seg_id] + 2 * c[seg_id] * dx + 3 * d[seg_id] * dx * dx;
}

double Spline::calc_dd(double t) {
    if (t < x.front() || t > x.back()) {
        throw std::invalid_argument("received value out of the pre-defined range");
    }
    int seg_id = bisect(t, 0, nx);
    double dx = t - x[seg_id];
    return 2 * c[seg_id] + 6 * d[seg_id] * dx;
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
    for (int i = 1; i < B.size() - 1; i++)
        B(i) = (y[i + 1] - y[i]) / h[i] - (y[i] - y[i - 1]) / h[i - 1];
    return B;
}

int Spline::bisect(double t, int start, int end) {
    int mid = (start + end) / 2;
    if (t == x[mid] || end - start <= 1) {
        return mid;
    } else if (t > x[mid]) {
        return bisect(t, mid, end);
    } else {
        return bisect(t, start, mid);
    }
}
Spline::Spline(const std::vector<double> &x_, const std::vector<double> &y_) : x(x_), y(y_), nx(x_.size() - 1), h(vec_diff(x_)) {
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
    // 套公式求出系数b和d
    a = y;
    a.pop_back();//a为n-1个
    c = m;
    c.pop_back();//c为n-1个
    b.resize(nx);
    d.resize(nx);
    for (int i = 0; i < nx; i++) {
        d.emplace_back((m[i + 1] - m[i]) / (6 * h[i]));
        b.emplace_back((y[i + 1] - y[i]) / h[i] - h[i] * m[i] / 2 - (m[i + 1] - m[i]) * h[i] / 6);
    }
}


Point Spline2D::calc_position(double s_t) {
    Point point;
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
Spline2D::Spline2D(const std::vector<double> &x, const std::vector<double> &y) {
    if (x.size() != y.size()) {
        std::cerr << "x与y的数量不相等!" << std::endl;
    }
    s = calc_s(x, y);
    sx = Spline(s, x);
    sy = Spline(s, y);
}
double Spline2D::length() { return s.back(); }


void CubicSplineClass::UpdateKeeper(common_private_msgs::vehicle_status last_back) {
    keeper[0].x = mystate.x;
    keeper[0].y = mystate.y;
    keeper[0].yaw = mystate.yaw;
    keeper[1].x = last_back.xPos;
    keeper[1].y = last_back.yPos;
    keeper[1].yaw = last_back.yaw;
}

int CubicSplineClass::findMatchpoint(double x, double y, CSRefPath &input_trajectory) {
    int num = input_trajectory.r_curvature.size();
    double dis_min = std::numeric_limits<double>::max();
    int index = 0;
    for (int i = 0; i < num; i++) {
        double temp_dis = std::pow(input_trajectory.r_x[i] - x, 2) + std::pow(input_trajectory.r_y[i] - y, 2);
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
