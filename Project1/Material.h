#pragma once
#include"RayTraceMath.h"
#include "Hitable.h"
#include "Texture.h"
class material
{
public:
    virtual bool scatter(const ray& r_in, const hit_record& rec, vec3& attenuation, ray& scattered,double& pdf) const 
    {
        return false;
    }

    virtual color emitted(const ray& r_in, const hit_record& rec, double u, double v, const point3& p) const
    {
        return color(0, 0, 0);
    }

    virtual double scattering_pdf(const ray& r_in, const hit_record& rec, const ray& scattered) const 
    {
        return 0;
    }

    virtual bool isLighting()
    {
        return false;
    }
};

class isotropic : public material {
public:
    isotropic(color c) : albedo(make_shared<solid_color>(c)) {}
    isotropic(shared_ptr<texture> a) : albedo(a) {}

    virtual bool scatter(
        const ray& r_in, const hit_record& rec, color& attenuation, ray& scattered, double& pdf
    ) const override {
        scattered = ray(rec.point, random_in_unit_sphere(), r_in.time());
        attenuation = albedo->value(rec.u, rec.v, rec.point);
        return true;
    }

public:
    shared_ptr<texture> albedo;
};

class lambertian :public material
{
public:
    lambertian(const color& a) :albedo(make_shared<solid_color>(a)) {}
    lambertian(shared_ptr<texture> a):albedo(a){}
    virtual bool scatter(const ray& r_in, const hit_record& rec, color& attenuation, ray& scattered, double& pdf) const
    {
        //vec3 scatter_direction = rec.normal + random_unit_vector();//随机采样方向
       
        //double theta = asin(pow(random_double(),0.5));
        //double phi = random_double() * 2 * pi;
        //vec3 sampleDir = vec3(sin(theta)*cos(phi),cos(theta),sin(theta)*sin(phi));//符合cos分布的采样方向
        //sampleDir = unit_vector(sampleDir);
        ////根据法线做旋转
        //if (dot(sampleDir, rec.normal) < 0)
        //    sampleDir *= -1;
       
        onb tbn;
        tbn.build_from_w(rec.normal);
        vec3 sampleDir = random_cosine_direction();
        sampleDir= tbn.local(sampleDir);
        scattered = ray(rec.point, sampleDir,r_in.time());
       
        attenuation = albedo->value(rec.u,rec.v,rec.point);
        pdf = dot(unit_vector(rec.normal), unit_vector(scattered.direction())) / pi;
        return true;
    }
    virtual double scattering_pdf(const ray& r_in, const hit_record& rec, const ray& scattered) const
    {
        double cosTheta = dot(unit_vector(rec.normal), unit_vector(scattered.direction()));
        return cosTheta < 0 ? 0 : cosTheta/pi;
    }
public:
    shared_ptr<texture> albedo;
};

class metal :public material
{
public:
    metal(const color& a, double g) :albedo(a), glossy(g) {}

    virtual bool scatter(const ray& r_in, const hit_record& rec, vec3& attenuation, ray& scattered, double& pdf) const
    {
        vec3 reflected = reflect(unit_vector(r_in.direction()), rec.normal);
        scattered = ray(rec.point, reflected + glossy * random_unit_vector(), r_in.time());
        attenuation = albedo;
        return (dot(scattered.direction(), rec.normal) > 0);
    }
public:
    color albedo;
    double  glossy;
};

class dielectric :public material
{
public:
    dielectric(double ri) :ref_idx(ri) {}
    virtual bool scatter(const ray& r_in, const hit_record& rec, vec3& attenuation, ray& scattered, double& pdf) const
    {
        attenuation = color(1.0, 1.0, 1.0);
        double eta = rec.front_face ? (1.0 / ref_idx) : ref_idx;
        vec3 unit_dir = unit_vector(r_in.direction());
        double cosTheta = fmin(dot(-unit_dir, rec.normal), 1.0);
        double sinTheta = sqrt(1.0 - cosTheta * cosTheta);

        //全内反射
        if (eta * sinTheta > 1.0)
        {
            vec3 reflected = reflect(unit_dir, rec.normal);
            scattered = ray(rec.point, reflected, r_in.time());
            return true;
        }

        //反射率（菲尼尔？）
        double relect_prob = schlick(cosTheta, eta);
        if (random_double() < relect_prob)
        {
            vec3 reflected = reflect(unit_dir, rec.normal);
            scattered = ray(rec.point, reflected, r_in.time());
            return true;
        }

        //折射
        vec3 refracted = refract(unit_dir, rec.normal, eta);
        scattered = ray(rec.point, refracted, r_in.time());
        return true;
    }


public:
    double ref_idx;
};

class diffuse_light : public material {
public:
    diffuse_light(shared_ptr<texture> a) : emit(a) {}
    diffuse_light(color c) : emit(make_shared<solid_color>(c)) {}

    virtual bool scatter(
        const ray& r_in, const hit_record& rec, color& attenuation, ray& scattered, double& pdf
    ) const override {
        return false;
    }

    virtual color emitted(const ray& r_in, const hit_record& rec, double u, double v, const point3& p) const override
    {
        if (rec.front_face)
            return emit->value(u, v, p);
        else
            return color(0, 0, 0);
    }

    virtual bool isLighting()
    {
        return true;
    }


public:
    shared_ptr<texture> emit;
};