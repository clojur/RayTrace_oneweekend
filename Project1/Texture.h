#pragma once
#include "RayTraceMath.h"
#include "perlin.h"

#include "stb_image.h"


//const unsigned char* loadfile(const std::string& file, int& size)
//{
//    std::ifstream fs(file.c_str(), std::ios::binary);
//    fs.seekg(0, std::ios::end);
//    size = fs.tellg();
//    char* data = new char[size + 1];
//    fs.seekg(0);
//    fs.read(data, size);
//    fs.close();
//    data[size] = 0;
//    return (unsigned char*)data;
//
//    const unsigned char* data = loadfile("D:/1.jpg", size);
//    const unsigned char* logo = stbi_load_from_memory(data, size, &w, &h, &channels, 0);
//}

using namespace std;
class texture
{
public:
	virtual color value(double u,double v,const point3& p) const = 0;
};

class solid_color :public texture
{
public:
	solid_color(){}
	solid_color(color c):color_value(c){}

	solid_color(double red, double green, double blue)
		:solid_color(color(red, green, blue)) {}

	virtual color value(double u, double v, const point3& p) const override
	{
		return color_value;
	}
private:
	color color_value;
};

class checker_texture : public texture {
public:
    checker_texture() {}

    checker_texture(shared_ptr<texture> _even, shared_ptr<texture> _odd)
        : even(_even), odd(_odd) {}

    checker_texture(color c1, color c2)
        : even(make_shared<solid_color>(c1)), odd(make_shared<solid_color>(c2)) {}

    virtual color value(double u, double v, const point3& p) const override {
        auto sines = sin(10 * p.x()) * sin(10 * p.y()) * sin(10 * p.z());
        if (sines < 0)
            return odd->value(u, v, p);
        else
            return even->value(u, v, p);
    }

public:
    shared_ptr<texture> odd;
    shared_ptr<texture> even;
};

class noise_texture : public texture {
public:
    enum NoiseType
    {
        NT_perlin,
        NT_turb,
        NT_marbled
    };
    noise_texture() { scale = 1.0; }
    noise_texture(double sc, NoiseType t) :scale(sc),noiseType(t) {}
    virtual color value(double u, double v, const point3& p) const override {

        color res(0,0,0);
        switch (noiseType)
        {
        case NoiseType::NT_perlin:
            res= color(1, 1, 1) * 0.5 * (1.0 + noise.noise(scale * p));
            break;
        case NoiseType::NT_turb:
            res= turbFun(u,v,p);
            break;
        case NoiseType::NT_marbled:
            res= marbledFun(u,v,p);
            break;
        default:
            break;
        }
        
        return res;
    }

    color turbFun(double u, double v, const point3& p) const
    {
        return color(1, 1, 1) * noise.turb(scale * p);
    }

    color marbledFun(double u, double v, const point3& p) const
    {
        return color(1, 1, 1) * 0.5 * (1 + sin(scale * p.z() + 10 * noise.turb(p)));
    }

public:
    perlin noise;
    double scale;
    NoiseType noiseType;
};

class image_texture : public texture {
public:
    const static int bytes_per_pixel = 3;

    image_texture()
        : data(nullptr), width(0), height(0), bytes_per_scanline(0) {}

    image_texture(const char* filename);

    ~image_texture() {
        delete data;
    }

    virtual color value(double u, double v, const vec3& p) const override;

private:
    unsigned char* data;
    int width, height;
    int bytes_per_scanline;
};