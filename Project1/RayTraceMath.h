#pragma once
#include<limits>
#include<cstdlib>
#include<cmath>
#include<iostream>
const double infinity = std::numeric_limits<double>::infinity();
const double pi = 3.1415926;
const unsigned int maxDepth = 50;
inline double degrees_to_radians(double degrees)
{
    return degrees * pi / 180.0;
}

inline double radians_to_degrees(double radians)
{
    return radians * 180 / pi;
}

inline double random_double()
{
    // Returns a random real in [0,1).
    return rand() / (RAND_MAX + 1.0);
}

inline double random_double(double min, double max)
{
    // Returns a random real in [min,max).
    return min + (max - min) * random_double();
}

inline double clamp(double value,double min, double max)
{
    if (value > max)
        return max;
    if (value < min)
        return min;
    return value;
}

class vec3 {
public:
    vec3() : e{ 0,0,0 } {}
    vec3(double e0, double e1, double e2) : e{ e0, e1, e2 } {}

    double x() const { return e[0]; }
    double y() const { return e[1]; }
    double z() const { return e[2]; }

    vec3 operator-() const { return vec3(-e[0], -e[1], -e[2]); }
    double operator[](int i) const { return e[i]; }
    double& operator[](int i) { return e[i]; }

    vec3& operator+=(const vec3& v) {
        e[0] += v.e[0];
        e[1] += v.e[1];
        e[2] += v.e[2];
        return *this;
    }

    vec3& operator*=(const double t) {
        e[0] *= t;
        e[1] *= t;
        e[2] *= t;
        return *this;
    }

    vec3& operator/=(const double t) {
        return *this *= 1 / t;
    }

    static vec3 random()
    {
        return vec3(random_double(), random_double(), random_double());
    }

    static vec3 random(double min, double max)
    {
        return vec3(random_double(min, max), random_double(min, max), random_double(min, max));
    }

    double length() const {
        return sqrt(length_squared());
    }

    double length_squared() const {
        return e[0] * e[0] + e[1] * e[1] + e[2] * e[2];
    }

public:
    double e[3];
};

using color = vec3;

// vec3 Utility Functions

inline std::ostream& operator<<(std::ostream& out, const vec3& v) {
    return out << v.e[0] << ' ' << v.e[1] << ' ' << v.e[2];
}

inline vec3 operator+(const vec3& u, const vec3& v) {
    return vec3(u.e[0] + v.e[0], u.e[1] + v.e[1], u.e[2] + v.e[2]);
}

inline vec3 operator-(const vec3& u, const vec3& v) {
    return vec3(u.e[0] - v.e[0], u.e[1] - v.e[1], u.e[2] - v.e[2]);
}

inline vec3 operator*(const vec3& u, const vec3& v) {
    return vec3(u.e[0] * v.e[0], u.e[1] * v.e[1], u.e[2] * v.e[2]);
}

inline vec3 operator*(double t, const vec3& v) {
    return vec3(t * v.e[0], t * v.e[1], t * v.e[2]);
}

inline vec3 operator*(const vec3& v, double t) {
    return t * v;
}

inline vec3 operator/(vec3 v, double t) {
    return (1 / t) * v;
}

inline double dot(const vec3& u, const vec3& v) {
    return u.e[0] * v.e[0]
        + u.e[1] * v.e[1]
        + u.e[2] * v.e[2];
}

inline vec3 cross(const vec3& u, const vec3& v) {
    return vec3(u.e[1] * v.e[2] - u.e[2] * v.e[1],
        u.e[2] * v.e[0] - u.e[0] * v.e[2],
        u.e[0] * v.e[1] - u.e[1] * v.e[0]);
}

inline vec3 unit_vector(vec3 v) {
    return v / v.length();
}

inline vec3 reflect(const vec3& v, const vec3& n)
{
    return v - 2 * dot(v, n) * n;
}

inline vec3 refract(const vec3& i, const vec3& n, double eta)
{
    double cos_theta = dot(-i, n);
    vec3 refract_out_verticle = eta * (i + cos_theta * n);
    vec3 refract_out_parallel = -sqrt(fabs(1.0 - refract_out_verticle.length_squared())) * n;
    return refract_out_verticle + refract_out_parallel;
}

inline double schlick(double cosine, double ref_idx)
{
    double r0 = (1 - ref_idx) / (1 + ref_idx);
    r0 *= r0;
    return r0 + (1 - r0) * pow((1 - cosine), 5);
}

inline vec3 random_in_unit_sphere()
{
    while (true)
    {
        vec3 sphereVec = vec3::random(-1, 1);
        if (sphereVec.length() >= 1.0)
            continue;

        return sphereVec;
    }
}

inline vec3 random_unit_vector()
{
    double a = random_double(0, 2 * pi);
    double z = random_double(-1, 1);
    double r = sqrt(1 - z * z);

    return vec3(r * cos(a), r * sin(a), z);
}

inline vec3 random_in_normal_half_sphere(const vec3& normal)
{
    while (true)
    {
        vec3 sphereVec = random_unit_vector();
        //判断是否在法线正半求
        bool isPositive = dot(unit_vector(sphereVec), normal) >= 0;
        if (isPositive)
            return sphereVec;
        else
            return -sphereVec;
    }
}

inline vec3 random_cosine_direction() {
    auto r1 = random_double();
    auto r2 = random_double();
    auto z = sqrt(1 - r2);

    auto phi = 2 * pi * r1;
    auto x = cos(phi) * sqrt(r2);
    auto y = sin(phi) * sqrt(r2);

    return vec3(x, y, z);
}

inline vec3 random_unit_disk()
{
    while (true)
    {
        vec3 res = vec3(random_double(-1, 1), random_double(-1, 1), 0);

        if (res.length_squared() < 1.0)
            return res;
    }
}

inline int random_int(int min, int max)
{
    return static_cast<int>(random_double(min,max));
}

using color = vec3;
using point3 = vec3;

class ray {
public:
    ray() {}
    ray(const vec3& origin, const vec3& direction,double time=0.0)
        : o(origin), dir(direction),tm(time)
    {
        //dir = unit_vector(dir);
    }

    vec3 origin() const { return o; }
    vec3 direction() const { return dir; }

    vec3 at(double t) const {
        return o + t * dir;
    }

    double time() const { return tm; }

public:
    vec3 o;
    vec3 dir;
    double tm;
};

class material;

struct hit_record
{
    vec3 point; //交点
    vec3 normal;
    bool front_face;
    double t; //距离
    double u;
    double v;
    std::shared_ptr<material> mat_ptr;

    inline void set_surface_normal(const ray& ray, const vec3& outward_normal)
    {
        if (dot(ray.dir, outward_normal) < 0)
        {
            front_face = true;
            normal = outward_normal;
        }
        else
        {
            front_face = false;
            normal = -outward_normal;
        }
    }
};

class aabb;
class hittable
{
public:
    virtual bool hit(const ray& r, double t_min, double t_max, hit_record& record) const = 0;
    virtual bool bounding_box(double time0, double time1, aabb& output_box) const = 0;
    virtual double pdf_value(const point3& o, const vec3& v) const 
    {
        return 0.0;
    }

    virtual vec3 random(const vec3& o) const 
    {
        return vec3(1, 0, 0);
    }
};

class aabb
{
public:
    aabb() {}
    aabb(const point3& a, const point3& b)
    {
        minimum = a;
        maximum = b;
    }

    point3 min() const { return minimum; }
    point3 max() const { return maximum; }

    bool hit(const ray& r, double t_min, double t_max) const
    {
        //for (int a = 0; a < 3; a++)
        //{
        //	auto t0 = fmin((minimum[a]-r.origin()[a])/r.direction()[a],
        //					(maximum[a]-r.origin()[a]/r.direction()[a]));

        //	auto t1 = fmax((minimum[a] - r.origin()[a]) / r.direction()[a],
        //		(maximum[a] - r.origin()[a] / r.direction()[a]));

        //	t_min = fmax(t0,t_min);
        //	t_max = fmin(t1,t_max);
        //	if (t_max <= t_min)
        //		return false;
        //}

        //return true;

        for (int a = 0; a < 3; a++) {
            auto invD = 1.0f / r.direction()[a];
            auto t0 = (min()[a] - r.origin()[a]) * invD;
            auto t1 = (max()[a] - r.origin()[a]) * invD;
            if (invD < 0.0f)
                std::swap(t0, t1);
            t_min = t0 > t_min ? t0 : t_min;
            t_max = t1 < t_max ? t1 : t_max;
            if (t_max <= t_min)
                return false;
        }
        return true;
    }

    static aabb surrounding_box(aabb box0, aabb box1)
    {
        point3 boxMin = vec3(fmin(box0.min().x(), box1.min().x()),
            fmin(box0.min().y(), box1.min().y()),
            fmin(box0.min().z(), box1.min().z()));

        point3 boxMax = vec3(fmax(box0.max().x(), box1.max().x()),
            fmax(box0.max().y(), box1.max().y()),
            fmax(box0.max().z(), box1.max().z()));

        return aabb(boxMin, boxMax);
    }

    point3 minimum;
    point3 maximum;
};

inline bool box_compare(const std::shared_ptr<hittable> a, const std::shared_ptr<hittable> b, int axis)
{
    aabb box_a;
    aabb box_b;

    if (!a->bounding_box(0, 0, box_a) || !b->bounding_box(0, 0, box_b))
        std::cerr << "No bounding box in bvh_node constructor.\n";

    return box_a.min().e[axis] < box_b.min().e[axis];
}

inline bool box_x_compare(const std::shared_ptr<hittable> a, const std::shared_ptr<hittable> b)
{
    return box_compare(a, b, 0);
}

inline bool box_y_compare(const std::shared_ptr<hittable> a, const std::shared_ptr<hittable> b)
{
    return box_compare(a, b, 1);
}

inline bool box_z_compare(const std::shared_ptr<hittable> a, const std::shared_ptr<hittable> b)
{
    return box_compare(a, b, 2);
}

class onb {
public:
    onb() {}

    inline vec3 operator[](int i) const { return axis[i]; }

    vec3 u() const { return axis[0]; }
    vec3 v() const { return axis[1]; }
    vec3 w() const { return axis[2]; }

    vec3 local(double a, double b, double c) const {
        return a * u() + b * v() + c * w();
    }

    vec3 local(const vec3& a) const {
        return a.x() * u() + a.y() * v() + a.z() * w();
    }

    void build_from_w(const vec3&n)
    {
        axis[2] = unit_vector(n);
        vec3 a = (fabs(w().x()) > 0.9) ? vec3(0, 1, 0) : vec3(1, 0, 0);
        axis[1] = unit_vector(cross(w(), a));
        axis[0] = cross(w(), v());
    }

public:
    vec3 axis[3];
};

class pdf {
public:
    virtual ~pdf() {}

    virtual double value(const vec3& direction) const = 0;
    virtual vec3 generate() const = 0;
};

class cosine_pdf : public pdf {
public:
    cosine_pdf(const vec3& w) { uvw.build_from_w(w); }

    virtual double value(const vec3& direction) const override
    {
        auto cosine = dot(unit_vector(direction), uvw.w());
        return (cosine <= 0) ? 0 : cosine / pi;
    }

    virtual vec3 generate() const override {
        return uvw.local(random_cosine_direction());
    }

public:
    onb uvw;
};

class mixture_pdf : public pdf {
public:
    mixture_pdf(std::shared_ptr<pdf> p0, std::shared_ptr<pdf> p1) {
        p[0] = p0;
        p[1] = p1;
    }

    virtual double value(const vec3& direction) const override {
        return 0.5 * p[0]->value(direction) + 0.5 * p[1]->value(direction);
    }

    virtual vec3 generate() const override {
        if (random_double() < 0.5)
            return p[0]->generate();
        else
            return p[1]->generate();
    }

public:
    std::shared_ptr<pdf> p[2];
};