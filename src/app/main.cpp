// \file main.cpp
// \author Bean
// \created 2020/02/09/22:04
// \update 2020/02/10/15:35

#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <fusion.h>
#include "MeanShift.h"
#include <iostream>

using namespace mosek::fusion;
using namespace monty;

// Input bounding boxes
std::string IN_PATH = "in.xywh";
// Regularized bounding boxes
std::string OUT_PATH = "out.xywh";
// The lower bound
const double delta_x = 5;
const double delta_y = 5;
const double delta_w = 3;
const double delta_h = 3;
// The weights
const double alpha_x = 100;
const double alpha_y = 100;
const double alpha_h = 1;
const double alpha_w = 1;

int N, m, n, m_, n_;

std::string read_file(const std::string& path);

void split(std::string& s, const std::string& p, std::vector<std::string>& r);

void get_xywh(std::string & s, std::vector<double> & x, std::vector<double> & y, std::vector<double> & w, std::vector<double> & h);

//void get_XYWH(std::string& s, std::vector<double>& X, std::vector<double>& Y, std::vector<double>& W, std::vector<double>& H);

void pre_cluster(std::vector<double>& x, std::vector<double>& X, double delta);

void regularize(std::vector<double> & x, std::vector<double> & y, std::vector<double> & w, std::vector<double> & h, std::vector<double> & X, std::vector<double> & Y, std::vector<double> & W, std::vector<double> & H, std::vector<int> & r);

void write_file(std::vector<double>& X, std::vector<double>& Y, std::vector<double>& W, std::vector<double>& H, std::vector<int>& r, const std::string& path);

int main(int argc, char** argv) {
    std::vector<double> x, y, w, h, X, Y, W, H;
    std::vector<int> r;
    IN_PATH = H2O_DATA_DIR + IN_PATH;
    OUT_PATH = H2O_DATA_DIR + OUT_PATH;

    auto s1 = read_file(IN_PATH);
    get_xywh(s1, x, y, w, h);

    pre_cluster(x, X, delta_x);
    pre_cluster(y, Y, delta_y);
    pre_cluster(w, W, delta_w);
    pre_cluster(h, H, delta_h);

    N = x.size();
    m = X.size();
    n = Y.size();
    m_ = W.size();
    n_ = H.size();

    regularize(x, y, w, h, X, Y, W, H, r);
    write_file(X, Y, W, H, r, OUT_PATH);

    return 0;
}

std::string read_file(const std::string& path) {
    std::ifstream file_stream(path, std::ios::in);
    assert(file_stream.is_open());
    std::ostringstream oss;
    oss << file_stream.rdbuf();
    auto s = oss.str();
    file_stream.close();
    return s;
}

void split(std::string& s, const std::string& p, std::vector<std::string>& r) {
    s += p;
    const int size = s.size();
    for (int i = 0; i < size; i++) {
        const std::string::size_type pos = s.find(p, i);
        if (pos < size) {
            std::string ss = s.substr(i, pos - i);
            r.push_back(ss);
            i = p.size() + pos - 1;
        }
    }
}

void get_xywh(std::string& s, std::vector<double>& x, std::vector<double>& y, std::vector<double>& w, std::vector<double>& h) {
    std::vector<std::string> str;
    split(s, "\n", str);
    for (auto& i : str) {
        std::vector<std::string> ss;
        split(i, " ", ss);
        x.push_back(atof(ss[0].c_str()));
        y.push_back(atof(ss[1].c_str()));
        w.push_back(atof(ss[2].c_str()));
        h.push_back(atof(ss[3].c_str()));
    }
}

//void get_XYWH(std::string& s, std::vector<double>& X, std::vector<double>& Y, std::vector<double>& W, std::vector<double>& H) {
//	std::vector<std::string> ss, x, y, w, h;
//	split(s, "\n", ss);
//	split(ss[0], " ", x);
//	split(ss[1], " ", y);
//	split(ss[2], " ", w);
//	split(ss[3], " ", h);
//	for (auto& i : x) {
//		X.push_back(atof(i.c_str()));
//	}
//	for (auto& i : y) {
//		Y.push_back(atof(i.c_str()));
//	}
//	for (auto& i : w) {
//		W.push_back(atof(i.c_str()));
//	}
//	for (auto& i : h) {
//		H.push_back(atof(i.c_str()));
//	}
//}

void regularize(std::vector<double>& x, std::vector<double>& y, std::vector<double>& w, std::vector<double>& h, std::vector<double>& X, std::vector<double>& Y, std::vector<double>& W, std::vector<double>& H, std::vector<int>& r) {
    int idx = N * (m + n + m_ + n_);
    int count = N * (m + n + m_ + n_) + (m + n + m_ + n_);
    // 目标函数的所有系数
    std::vector<double> a1;
    for (std::size_t i = 0; i < N; ++i) {
        for (std::size_t k = 0; k < m; ++k) {
            double t = (x.at(i) - X.at(k)) * (x.at(i) - X.at(k));
            a1.push_back(t);
        }
    }
    for (std::size_t i = 0; i < N; ++i) {
        for (std::size_t k = 0; k < n; ++k) {
            double t = (y.at(i) - Y.at(k)) * (y.at(i) - Y.at(k));
            a1.push_back(t);
        }
    }
    for (std::size_t i = 0; i < N; ++i) {
        for (std::size_t k = 0; k < m_; ++k) {
            double t = (w.at(i) - W.at(k)) * (w.at(i) - W.at(k));
            a1.push_back(t);
        }
    }
    for (std::size_t i = 0; i < N; ++i) {
        for (std::size_t k = 0; k < n_; ++k) {
            double t = (h.at(i) - H.at(k)) * (h.at(i) - H.at(k));
            a1.push_back(t);
        }
    }
    for (std::size_t i = 0; i < m; ++i) {
        a1.push_back(alpha_x);
    }
    for (std::size_t i = 0; i < n; ++i) {
        a1.push_back(alpha_y);
    }
    for (std::size_t i = 0; i < m_; ++i) {
        a1.push_back(alpha_w);
    }
    for (std::size_t i = 0; i < n_; ++i) {
        a1.push_back(alpha_h);
    }
    std::vector<double> v;
    // 任意系数向量与未知向量点乘小于等于1
    std::vector<std::vector<double>> a2;
    for (std::size_t i = 0; i < N; ++i) {
        v.clear();
        v.resize(count, 0);
        for (std::size_t k = 0; k < m; ++k) {
            v[i * m + k] = 1.0;
        }
        a2.push_back(v);
    }
    for (std::size_t i = 0; i < N; ++i) {
        v.clear();
        v.resize(count, 0);
        for (std::size_t k = 0; k < n; ++k) {
            v[N * m + i * n + k] = 1.0;
        }
        a2.push_back(v);
    }
    for (std::size_t i = 0; i < N; ++i) {
        v.clear();
        v.resize(count, 0);
        for (std::size_t k = 0; k < m_; ++k) {
            v[N * (m + n) + i * m_ + k] = 1.0;
        }
        a2.push_back(v);
    }
    for (std::size_t i = 0; i < N; ++i) {
        v.clear();
        v.resize(count, 0);
        for (std::size_t k = 0; k < n_; ++k) {
            v[N * (m + n) + N * m_ + i * n_ + k] = 1.0;
        }
        a2.push_back(v);
    }
    // 大于0
    std::vector<std::vector<double>> a3;
    for (std::size_t i = 0; i < m; ++i) {
        v.clear();
        v.resize(count, 0);
        v[idx + i] = -1.0;
        for (std::size_t k = 0; k < N; ++k) {
            v[k * m + i] = 1.0;
        }
        a3.push_back(v);
    }
    for (std::size_t i = 0; i < n; ++i) {
        v.clear();
        v.resize(count, 0);
        v[idx + m + i] = -1.0;
        for (std::size_t k = 0; k < N; ++k) {
            v[N * m + k * n + i] = 1.0;
        }
        a3.push_back(v);
    }
    for (std::size_t i = 0; i < m_; ++i) {
        v.clear();
        v.resize(count, 0);
        v[idx + m + n + i] = -1.0;
        for (std::size_t k = 0; k < N; ++k) {
            v[N * m + N * n + k * m_ + i] = 1.0;
        }
        a3.push_back(v);
    }
    for (std::size_t i = 0; i < n_; ++i) {
        v.clear();
        v.resize(count, 0);
        v[idx + m + n + m_ + i] = -1.0;
        for (std::size_t k = 0; k < N; ++k) {
            v[N * m + N * n + N * m_ + k * n_ + i] = 1.0;
        }
        a3.push_back(v);
    }
    // 小于0
    std::vector<std::vector<double>> a4;
    for (std::size_t i = 0; i < m; ++i) {
        for (std::size_t k = 0; k < N; ++k) {
            v.clear();
            v.resize(count, 0);
            v[k * m + i] = 1.0;
            v[idx + i] = -1.0;
            a4.push_back(v);
        }
    }
    for (std::size_t i = 0; i < n; ++i) {
        for (std::size_t k = 0; k < N; ++k) {
            v.clear();
            v.resize(count, 0);
            v[N * m + k * n + i] = 1.0;
            v[idx + m + i] = -1.0;
            a4.push_back(v);
        }
    }
    for (std::size_t i = 0; i < m_; ++i) {
        for (std::size_t k = 0; k < N; ++k) {
            v.clear();
            v.resize(count, 0);
            v[N * m + N * n + k * m_ + i] = 1.0;
            v[idx + m + n + i] = -1.0;
            a4.push_back(v);
        }
    }
    for (std::size_t i = 0; i < n_; ++i) {
        for (std::size_t k = 0; k < N; ++k) {
            v.clear();
            v.resize(count, 0);
            v[N * m + N * n + N * m_ + k * n_ + i] = 1.0;
            v[idx + m + n + m_ + i] = -1.0;
            a4.push_back(v);
        }
    }
    int total = 0;
    Model::t M = new Model("lo1");
    auto _M = finally([&]() { M->dispose(); });
    Variable::t x_ = M->variable("x", count, Domain::integral(Domain::greaterThan(0.0)));
    M->constraint(x_, Domain::lessThan(1.0));
    for (std::size_t i = 0; i < a2.size(); ++i) {
        auto tmp = new_array_ptr<double>(a2.at(i));
        M->constraint(std::to_string(total), Expr::dot(tmp, x_), Domain::equalsTo(1.0));
        total++;
    }
    for (std::size_t i = 0; i < a3.size(); ++i) {
        auto tmp = new_array_ptr<double>(a3.at(i));
        M->constraint(std::to_string(total), Expr::dot(tmp, x_), Domain::greaterThan(0.0));
        total++;
    }
    for (std::size_t i = 0; i < a4.size(); ++i) {
        auto tmp = new_array_ptr<double>(a4.at(i));
        M->constraint(std::to_string(total), Expr::dot(tmp, x_), Domain::lessThan(0.0));
        total++;
    }
    auto A = new_array_ptr<double>(a1);
    M->objective("obj", ObjectiveSense::Minimize, Expr::dot(A, x_));
    M->solve();
    auto sol = x_->level();
    for (double i : (*sol)) {
        r.push_back(static_cast<int>(i));
    }
}

void write_file(std::vector<double>& X, std::vector<double>& Y, std::vector<double>& W, std::vector<double>& H, std::vector<int>& r, const std::string& path) {
    std::vector<int> r_x, r_y, r_w, r_h;
    for (std::size_t i = 0; i < N; ++i) {
        for (std::size_t k = 0; k < m; ++k) {
            if (r.at(i * m + k) != 0) {
                r_x.push_back(k);
            }
        }
    }
    for (std::size_t i = 0; i < N; ++i) {
        for (std::size_t k = 0; k < n; ++k) {
            if (r.at(N * m + i * n + k) != 0) {
                r_y.push_back(k);
            }
        }
    }
    for (std::size_t i = 0; i < N; ++i) {
        for (std::size_t k = 0; k < m_; ++k) {
            if (r.at(N * m + N * n + i * m_ + k) != 0) {
                r_w.push_back(k);
            }
        }
    }
    for (std::size_t i = 0; i < N; ++i) {
        for (std::size_t k = 0; k < n_; ++k) {
            if (r.at(N * m + N * n + N * m_ + i * n_ + k) != 0) {
                r_h.push_back(k);
            }
        }
    }
    std::string result = "";
    for (std::size_t i = 0; i < N; ++i) {
        double x = X.at(r_x.at(i));
        double y = Y.at(r_y.at(i));
        double w = W.at(r_w.at(i));
        double h = H.at(r_h.at(i));
        result += std::to_string(x) + " " + std::to_string(y) + " " + std::to_string(w) + " " + std::to_string(h) + "\n";
    }
    std::ofstream out(path);
    out << result;
    out.close();
}

void pre_cluster(std::vector<double>& x, std::vector<double>& X, double delta) {
    std::vector<std::vector<double>> points;
    for (std::size_t i = 0; i < x.size(); ++i) {
        std::vector<double> p;
        p.push_back(x[i]);
        p.push_back(0);
        points.push_back(p);
    }
    MeanShift* ms = new MeanShift();
    std::vector<Cluster> clusters = ms->cluster(points, delta);
    for (auto& i : clusters) {
        X.push_back(i.mode[0]);
    }
}
