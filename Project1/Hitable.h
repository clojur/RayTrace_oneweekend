#pragma once
#include<vector>
#include<memory>

#include "RayTraceMath.h"
#include "Texture.h"
#include <algorithm>
using namespace std;
class material;


class sphere :public hittable
{
public:
    sphere() = delete;
    sphere(const vec3& c, const double r, std::shared_ptr<material> mat)
        :_center(c)
        , _radius(r)
        , mat_ptr(mat)
    {

    }
    virtual bool hit(const ray& r, double t_min, double t_max, hit_record& record) const override
    {
        vec3 oc = r.origin() - _center;
        auto a = dot(r.direction(), r.direction());
        auto half_b = dot(oc, r.direction());
        auto c = dot(oc, oc) - _radius * _radius;
        auto discriminant = half_b * half_b - a * c;
        if (discriminant < 0)  return false;

        auto sqrtd = sqrt(discriminant);

        auto root = (-half_b - sqrtd) / a;
        if (root<t_min || root>t_max)
        {
            root = (-half_b + sqrtd) / a;
            if (root<t_min || root>t_max)
                return false;
        }

        record.t = root;
        record.point = r.at(root);
        vec3 outward_normal = (record.point - _center) / _radius;
        record.set_surface_normal(r, outward_normal);
        get_sphere_uv(outward_normal, record.u, record.v);
        record.mat_ptr = mat_ptr;
        return true;
    }
    virtual bool bounding_box(double time0, double time1, aabb& output_box) const override
    {
        output_box = aabb(_center-vec3(_radius,_radius,_radius),
                          _center + vec3(_radius, _radius, _radius));
        return true;
    }
    static void get_sphere_uv(const point3& p, double& u, double& v)
    {
        auto theta = acos(-p.y());
        auto phi = atan2(-p.z(),p.x())+pi;

        u = phi / (2 * pi);
        v = theta / pi;
    }
public:
    vec3 _center;
    double _radius;
    std::shared_ptr<material> mat_ptr;
};

class moving_sphere :public hittable
{
public:
    moving_sphere(){}
    moving_sphere(point3 cen0, point3 cen1, double time0, double time1, double r, std::shared_ptr<material> m)
        :center0(cen0), center1(cen1)
        ,time0(time0),time1(time1)
        ,radius(r),mat_ptr(m)
    {}
    virtual bool hit(const ray& r, double t_min, double t_max, hit_record& rec)const override
    {
        vec3 oc = r.origin() - center(r.time());
        auto a = dot(r.direction(), r.direction());
        auto half_b = dot(oc, r.direction());
        auto c = dot(oc, oc) - radius * radius;
        auto discriminant = half_b * half_b - a * c;
        if (discriminant < 0)  
            return false;

        auto sqrtd = sqrt(discriminant);

        auto root = (-half_b - sqrtd) / a;
        if (root<t_min || root>t_max)
        {
            root = (-half_b + sqrtd) / a;
            if (root<t_min || root>t_max)
                return false;
        }

        rec.t = root;
        rec.point = r.at(root);
        vec3 outward_normal = (rec.point - center(r.time())) / radius;
        rec.set_surface_normal(r, outward_normal);
        rec.mat_ptr = mat_ptr;
        return true;
    }
    virtual bool bounding_box(double time0, double time1, aabb& output_box) const override
    {
        aabb box0(center(time0) - vec3(radius, radius, radius),
            center(time0) + vec3(radius, radius, radius));
        aabb box1(center(time1) - vec3(radius, radius, radius),
            center(time1) + vec3(radius, radius, radius));

        output_box = aabb::surrounding_box(box0,box1);
        return true;

    }
    point3 center(double time) const
    {
        return center0 + ((time - time0) / (time1 - time0)) * (center1 - center0);
    }
public:
    point3 center0, center1;
    double time0, time1;
    double radius;
    std::shared_ptr<material> mat_ptr;
};

class xy_rect:public hittable
{
public:
    xy_rect(double x0,double x1,double y0,double y1,double z, std::shared_ptr<material> m)
        :x0(x0),y0(y0),x1(x1),y1(y1),z(z),mt(m)
    {
    }
    virtual bool hit(const ray& r, double t_min, double t_max, hit_record& record) const override
    {
        double t = (z - r.origin().z()) / r.direction().z();
        if (t<t_min || t>t_max)
            return false;

        double x = r.origin().x() + t * r.direction().x();
        double y = r.origin().y() + t * r.direction().y();

        if (x<x0 || x>x1 || y<y0 || y>y1)
            return false;
        
        vec3 outward_normal = vec3(0,0,1);
        record.mat_ptr = mt;
        record.u = (x - x0) / (x1 - x0);
        record.v = (y - y0) / (y1 - y0);
        record.t = t;
        record.point = r.at(t);
        record.set_surface_normal(r, outward_normal);
        return true;
    }

    virtual bool bounding_box(double time0, double time1, aabb& output_box) const override
    {
        output_box = aabb(point3(x0,y0,z-0.0001),point3(x1,y1,z+0.0001));
        return true;
    }

    double x0, x1, y0, y1,z;
    std::shared_ptr<material> mt;
};

class xz_rect :public hittable
{
public:
    xz_rect(double x0, double x1, double z0, double z1, double y, std::shared_ptr<material> m)
        :x0(x0), z0(z0), x1(x1), z1(z1), y(y), mt(m)
    {
    }
    virtual bool hit(const ray& r, double t_min, double t_max, hit_record& record) const override
    {
        double t = (y - r.origin().y()) / r.direction().y();
        if (t<t_min || t>t_max)
            return false;

        double x = r.origin().x() + t * r.direction().x();
        double z = r.origin().z() + t * r.direction().z();

        if (x<x0 || x>x1 || z<z0 || z>z1)
            return false;

        vec3 outward_normal = vec3(0, 1, 0);
        record.mat_ptr = mt;
        record.u = (x - x0) / (x1 - x0);
        record.v = (z - z0) / (z1 - z0);
        record.t = t;
        record.point = r.at(t);
        record.set_surface_normal(r, outward_normal);
        return true;
    }

    virtual bool bounding_box(double time0, double time1, aabb& output_box) const override
    {
        output_box = aabb(point3(x0, y - 0.0001, z0), point3(x1, y + 0.0001, z1));
        return true;
    }

    virtual double pdf_value(const point3& origin, const vec3& v) const override 
    {
        hit_record rec;
        if (!this->hit(ray(origin, v), 0.001, infinity, rec))
            return 0;

        auto area = (x1 - x0) * (z1 - z0);
        auto distance_squared = /*rec.t * rec.t * */v.length_squared();
        auto cosine = fabs(dot(v, rec.normal) / v.length());

        return distance_squared / (cosine * area);
    }

    virtual vec3 random(const point3& origin) const override 
    {
        auto random_point = point3(random_double(x0, x1), y, random_double(z0, z1));
        return random_point - origin;
    }

    double x0, x1, z0, z1, y;
    std::shared_ptr<material> mt;
};


class yz_rect:public hittable
{
public:
    yz_rect(double y0, double y1, double z0, double z1, double x, std::shared_ptr<material> m)
        :y0(y0), z0(z0), y1(y1), z1(z1), x(x), mt(m)
    {
    }
    virtual bool hit(const ray& r, double t_min, double t_max, hit_record& record) const override
    {
        double t = (x - r.origin().x()) / r.direction().x();
        if (t<t_min || t>t_max)
            return false;

        double y = r.origin().y() + t * r.direction().y();
        double z = r.origin().z() + t * r.direction().z();

        if (y<y0 || y>y1 || z<z0 || z>z1)
            return false;

        vec3 outward_normal = vec3(1, 0, 0);
        record.mat_ptr = mt;
        record.u = (y - y0) / (y1 - y0);
        record.v = (z - z0) / (z1 - z0);
        record.t = t;
        record.point = r.at(t);
        record.set_surface_normal(r, outward_normal);
        return true;
    }

    virtual bool bounding_box(double time0, double time1, aabb& output_box) const override
    {
        output_box = aabb(point3(x - 0.0001, y0 , z0), point3(x+ 0.0001, y1 , z1));
        return true;
    }

    double y0, y1, z0, z1, x;
    std::shared_ptr<material> mt;
};

class hittable_list :public hittable
{
public:
    hittable_list() {}
    hittable_list(std::shared_ptr<hittable> obj) { add(obj); }
    void clear() { objects.clear(); }
    void add(std::shared_ptr<hittable> obj) { objects.push_back(obj); }
    virtual bool hit(const ray& r, double t_min, double t_max, hit_record& record)const  override
    {
        hit_record temp_rec;
        bool hit_anything = false;
        auto closest_so_far = t_max;//用closest_so_far变量后，能找到最近点

        //遍历场景对象与射线求交
        for (const auto& object : objects) {
            if (object->hit(r, t_min, closest_so_far, temp_rec)) {
                hit_anything = true;
                closest_so_far = temp_rec.t;//每次求交都记录最大t，下次求交比它大的t都被否定，遍历完场景后record记录的即是最近交点
                record = temp_rec;
            }
        }

        return hit_anything;
    }
    virtual bool bounding_box(double time0, double time1, aabb& output_box) const override
    {
        if (objects.empty())
            return false;

        aabb temp_box;
        bool first_box = true;

        for (const auto& object : objects)
        {
            if (!object->bounding_box(time0, time1, temp_box))
                return false;

            output_box = first_box ? temp_box : aabb::surrounding_box(output_box,temp_box);

            first_box = false;
        }

        return true;
    }
public:
    std::vector<std::shared_ptr<hittable>> objects;
};

class bvh_node :public hittable
{
public:
    bvh_node() 
    {
    };

    bvh_node(const hittable_list& list, double time0, double time1) 
        :bvh_node(list.objects,0,list.objects.size(),time0,time1)
    {
    
    };

    bvh_node(const std::vector<std::shared_ptr<hittable>>& src_objects, size_t start, size_t end, double time0, double time1)
    {
        auto objects = src_objects;
        auto axis = random_int(0,2);
        auto comparator = (axis == 0) ? box_x_compare : (axis == 1) ? box_y_compare : box_z_compare;

        size_t object_span = end - start;

        if (object_span == 1)
        {
            left = right = objects[start];
        }
        else if (object_span == 2)
        {
            if (comparator(objects[start], objects[start + 1]))
            {
                left = objects[start];
                right = objects[start + 1];
            }
            else
            {
                left = objects[start + 1];
                right = objects[start];
            }
        }
        else
        {
            std::sort(objects.begin() + start, objects.begin() + end, comparator);

            auto mid = start + object_span / 2;

            left = std::make_shared<bvh_node>(objects,start,mid,time0,time1);
            right = std::make_shared<bvh_node>(objects, mid, end, time0, time1);
        }

        aabb box_left, box_right;
        if(!left->bounding_box(time0,time1,box_left)
            || !right->bounding_box(time0,time1,box_right))
            std::cerr<< "No bounding box in bvh_node constructor.\n";

        box = aabb::surrounding_box(box_left,box_right);
    }

    virtual bool hit(const ray& r, double t_min, double t_max, hit_record& record)const  override
    {
        if (!box.hit(r, t_min, t_max))
            return false;

        bool hit_left = left->hit(r, t_min, t_max, record);
        bool hit_right = right->hit(r, t_min, t_max, record);

        return hit_left || hit_right;
    }

    virtual bool bounding_box(double time0, double time1, aabb& output_box) const override
    {
        output_box = box;
        return true;
    }

    std::shared_ptr<hittable> left;
    std::shared_ptr<hittable> right;
    aabb box;

};

class box : public hittable {
public:
    box() {}
    box(const point3& p0, const point3& p1, shared_ptr<material> ptr)
    {
        box_min = p0;
        box_max = p1;

        sides.add(make_shared<xy_rect>(p0.x(), p1.x(), p0.y(), p1.y(), p1.z(), ptr));
        sides.add(make_shared<xy_rect>(p0.x(), p1.x(), p0.y(), p1.y(), p0.z(), ptr));

        sides.add(make_shared<xz_rect>(p0.x(), p1.x(), p0.z(), p1.z(), p1.y(), ptr));
        sides.add(make_shared<xz_rect>(p0.x(), p1.x(), p0.z(), p1.z(), p0.y(), ptr));

        sides.add(make_shared<yz_rect>(p0.y(), p1.y(), p0.z(), p1.z(), p1.x(), ptr));
        sides.add(make_shared<yz_rect>(p0.y(), p1.y(), p0.z(), p1.z(), p0.x(), ptr));
    }

    virtual bool hit(const ray& r, double t_min, double t_max, hit_record& rec) const override
    {
        return sides.hit(r, t_min, t_max, rec);
    }

    virtual bool bounding_box(double time0, double time1, aabb& output_box) const override {
        output_box = aabb(box_min, box_max);
        return true;
    }

public:
    point3 box_min;
    point3 box_max;
    hittable_list sides;
};

class translate : public hittable {
public:
    translate(shared_ptr<hittable> p, const vec3& displacement)
        : ptr(p), offset(displacement) {}

    virtual bool hit(
        const ray& r, double t_min, double t_max, hit_record& rec) const override
    {
        ray moved_r(r.origin() - offset, r.direction(), r.time());
        if (!ptr->hit(moved_r, t_min, t_max, rec))
            return false;

        rec.point += offset;
        rec.set_surface_normal(moved_r, rec.normal);

        return true;
    }

    virtual bool bounding_box(double time0, double time1, aabb& output_box) const override
    {
        if (!ptr->bounding_box(time0, time1, output_box))
            return false;

        output_box = aabb(
            output_box.min() + offset,
            output_box.max() + offset);

        return true;
    }

public:
    shared_ptr<hittable> ptr;
    vec3 offset;
};

class rotate_y : public hittable {
public:
    rotate_y(shared_ptr<hittable> p, double angle)
        :ptr(p)
    {
        auto radians = degrees_to_radians(angle);
        sin_theta = sin(radians);
        cos_theta = cos(radians);
        hasbox = ptr->bounding_box(0, 1, bbox);

        point3 min(infinity, infinity, infinity);
        point3 max(-infinity, -infinity, -infinity);

        for (int i = 0; i < 2; i++) 
        {
            for (int j = 0; j < 2; j++) 
            {
                for (int k = 0; k < 2; k++) 
                {
                    auto x = i * bbox.max().x() + (1 - i) * bbox.min().x();
                    auto y = j * bbox.max().y() + (1 - j) * bbox.min().y();
                    auto z = k * bbox.max().z() + (1 - k) * bbox.min().z();

                    auto newx = cos_theta * x + sin_theta * z;
                    auto newz = -sin_theta * x + cos_theta * z;

                    vec3 tester(newx, y, newz);

                    for (int c = 0; c < 3; c++) 
                    {
                        min[c] = fmin(min[c], tester[c]);
                        max[c] = fmax(max[c], tester[c]);
                    }
                }
            }
        }

        bbox = aabb(min, max);
    }

    virtual bool hit(
        const ray& r, double t_min, double t_max, hit_record& rec) const override
    {
        auto origin = r.origin();
        auto direction = r.direction();

        origin[0] = cos_theta * r.origin()[0] - sin_theta * r.origin()[2];
        origin[2] = sin_theta * r.origin()[0] + cos_theta * r.origin()[2];

        direction[0] = cos_theta * r.direction()[0] - sin_theta * r.direction()[2];
        direction[2] = sin_theta * r.direction()[0] + cos_theta * r.direction()[2];

        ray rotated_r(origin, direction, r.time());

        if (!ptr->hit(rotated_r, t_min, t_max, rec))
            return false;

        auto p = rec.point;
        auto normal = rec.normal;

        p[0] = cos_theta * rec.point[0] + sin_theta * rec.point[2];
        p[2] = -sin_theta * rec.point[0] + cos_theta * rec.point[2];

        normal[0] = cos_theta * rec.normal[0] + sin_theta * rec.normal[2];
        normal[2] = -sin_theta * rec.normal[0] + cos_theta * rec.normal[2];

        rec.point = p;
        rec.set_surface_normal(rotated_r, normal);

        return true;
    }

    virtual bool bounding_box(double time0, double time1, aabb& output_box) const override {
        output_box = bbox;
        return hasbox;
    }

public:
    shared_ptr<hittable> ptr;
    double sin_theta;
    double cos_theta;
    bool hasbox;
    aabb bbox;
};


class constant_medium : public hittable {
public:
    constant_medium(std::shared_ptr<hittable> b, double d, std::shared_ptr<texture> a);


    constant_medium(shared_ptr<hittable> b, double d, color c);


    virtual bool hit(const ray& r, double t_min, double t_max, hit_record& rec) const override;

    virtual bool bounding_box(double time0, double time1, aabb& output_box) const override {
        return boundary->bounding_box(time0, time1, output_box);
    }

public:
    shared_ptr<hittable> boundary;
    shared_ptr<material> phase_function;
    double neg_inv_density;
};

class flip_face : public hittable {
public:
    flip_face(shared_ptr<hittable> p) : ptr(p) {}

    virtual bool hit(
        const ray& r, double t_min, double t_max, hit_record& rec) const override {

        if (!ptr->hit(r, t_min, t_max, rec))
            return false;

        rec.front_face = !rec.front_face;
        return true;
    }

    virtual bool bounding_box(double time0, double time1, aabb& output_box) const override {
        return ptr->bounding_box(time0, time1, output_box);
    }

public:
    shared_ptr<hittable> ptr;
};

class hittable_pdf : public pdf {
public:
    hittable_pdf(shared_ptr<hittable> p, const point3& origin) : ptr(p), o(origin) {}

    virtual double value(const vec3& direction) const override {
        return ptr->pdf_value(o, direction);
    }

    virtual vec3 generate() const override {
        return ptr->random(o);
    }

public:
    point3 o;
    shared_ptr<hittable> ptr;
};