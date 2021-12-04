
#include<thread>
#include<mutex>
#include<iomanip>

#include "RayTraceMath.h"
#include "Material.h"
#include "Hitable.h"

struct CameraParam
{
	vec3 eye;
	vec3 target;
	vec3 up;
	double vFOV;
	double aspect;
	double aperture;
	double fous_dist;
	double time0;
	double time1;
};

class camera
{
public:
	camera() = delete;

	camera(const CameraParam& cp)
		:_eye(cp.eye)
		, aspect_ratio(cp.aspect)
		, vertical_FOV(cp.vFOV)
	{
		lens_radius = cp.aperture / 2;
		double h = tan(degrees_to_radians(cp.vFOV) * 0.5);
		double w = h * cp.aspect;

		double viewport_height = 2.0 * h;
		double viewport_widht = aspect_ratio * viewport_height;

		time0 = cp.time0;
		time1 = cp.time1;


		vec3 viewDir = unit_vector(cp.eye - cp.target);
		vec3 rightDir = unit_vector(cross(cp.up, viewDir));
		vec3 upDir = unit_vector(cross(viewDir, rightDir));

		horizontal = cp.fous_dist * rightDir * viewport_widht;
		vertical = cp.fous_dist * upDir * viewport_height;

		lower_left_corner = _eye - horizontal / 2.0 - vertical / 2.0 - cp.fous_dist * viewDir;
	}

	ray get_ray(float u, float v) const
	{
		vec3 rd = lens_radius * random_unit_disk();
		vec3 offset = horizontal * rd.x() + vertical * rd.y();
		return ray(_eye + offset, lower_left_corner + u * horizontal + v * vertical - _eye - offset, random_double(time0, time1));
	}

private:
	vec3 _eye;
	vec3 _lookat;
	vec3 _up;
	double aspect_ratio;
	double vertical_FOV;
	double lens_radius;
	vec3 vertical;
	vec3 horizontal;
	vec3 lower_left_corner;
	double time0, time1;
};

vec3 ray_color(const ray& r, hittable& world,shared_ptr<hittable> lights, const color& background,int depth) {
	hit_record rec;

	//double P_RR = 0.7;
	//if (random_double() > P_RR)
	//	return vec3(0, 0, 0);

	 if(depth<=0)
		return background;

	if (!world.hit(r, 0.001, infinity, rec))
		return background;

	ray scattered;
	color attenuation;
	color emitted = rec.mat_ptr->emitted(r,rec,rec.u, rec.v, rec.point);
	double pdf;
	color albedo;
	//如果交到光源或其他无散射的材质返回Le
	if (!rec.mat_ptr->scatter(r, rec, albedo, scattered, pdf))
		return emitted;

	//////////////////////////////////////////////
	/*（旧）直接采样光源*/
	//auto on_light = point3(random_double(213, 343), 554, random_double(227, 332));
	//auto to_light = on_light - rec.point;
	//auto distance_squared = to_light.length_squared();
	//to_light = unit_vector(to_light);

	////光源在面的背部，返回Le
	//if (dot(to_light, rec.normal) < 0)
	//	return emitted;

	//double light_area = (343 - 213) * (332 - 227);
	//auto light_cosine = fabs(to_light.y());
	////如果光源面对半球贡献特别小，返回Le
	//if (light_cosine < 0.000001)
	//	return emitted;

	//pdf = distance_squared / (light_cosine * light_area);
	//scattered = ray(rec.point, to_light, r.time());
	/////////////////////////////////////////////////////////

	/*直接采样光源*/
	hittable_pdf light_pdf(lights, rec.point);
	scattered = ray(rec.point, light_pdf.generate(), r.time());
	pdf = light_pdf.value(scattered.direction());

	if (pdf == 0)
		return background;

	/*cosine-weighted采样*/
	//cosine_pdf p(rec.normal);
	//scattered = ray(rec.point, p.generate(), r.time());
	//pdf = p.value(scattered.direction());

	/*混合PDF*/
	//auto p0 = make_shared<hittable_pdf>(lights, rec.point);
	//auto p1 = make_shared<cosine_pdf>(rec.normal);
	//mixture_pdf mixed_pdf(p0, p1);
	//scattered = ray(rec.point, mixed_pdf.generate(), r.time());
	//pdf = mixed_pdf.value(scattered.direction());

	return emitted+ albedo * rec.mat_ptr->scattering_pdf(r,rec,scattered)
					* ray_color(scattered, world,lights, background,depth-1) / pdf;
}
void write_color(std::ostream& out, const vec3& color) {
	// Write the translated [0,255] value of each color component.
	out << static_cast<int>(255.999 * clamp(color[0], 0.0, 0.99999)) << ' '
		<< static_cast<int>(255.999 * clamp(color[1], 0.0, 0.99999)) << ' '
		<< static_cast<int>(255.999 * clamp(color[2], 0.0, 0.99999)) << '\n';
}

using namespace std;
using point3 = vec3;
hittable_list random_scene() {
	hittable_list world;

	auto ground_material = make_shared<lambertian>(color(0.5, 0.5, 0.5));//地面
	auto checker = make_shared<checker_texture>(color(0.2, 0.2, 0.2), color(0.9, 0.9, 0.9));
	auto checker_material = make_shared<lambertian>(checker);
	world.add(make_shared<sphere>(point3(0, -1000, 0), 1000, checker_material));

	for (int a = -11; a < 11; a++) {
		for (int b = -11; b < 11; b++) {
			auto choose_mat = random_double(); //随机材质
			point3 center(a + 0.9 * random_double(), 0.2, b + 0.9 * random_double()); //随机中心

			if ((center - point3(4, 0.2, 0)).length() > 0.9) {
				shared_ptr<material> sphere_material;

				if (choose_mat < 0.4) {
					// diffuse 漫反射
					auto albedo = color::random() * color::random();
					sphere_material = make_shared<lambertian>(albedo);
					world.add(make_shared<sphere>(center, 0.2, sphere_material));
				}
				else if (choose_mat < 0.6)
				{
					// 空心玻璃
					sphere_material = make_shared<dielectric>(1.5);
					world.add(make_shared<sphere>(center, 0.2, sphere_material));
					world.add(make_shared<sphere>(center, -0.15, sphere_material));
				}
				else if (choose_mat < 0.95) {
					// metal 金属
					auto albedo = color::random(0.5, 1);
					auto fuzz = random_double(0, 0.5);
					sphere_material = make_shared<metal>(albedo, fuzz);
					world.add(make_shared<sphere>(center, 0.2, sphere_material));
				}
				else {
					// glass 玻璃
					sphere_material = make_shared<dielectric>(1.5);
					world.add(make_shared<sphere>(center, 0.2, sphere_material));
				}
			}
		}
	}

	auto material1 = make_shared<dielectric>(1.5);
	world.add(make_shared<sphere>(point3(0, 1, 0), 1.0, material1));

	auto material2 = make_shared<lambertian>(color(0.4, 0.2, 0.1));
	world.add(make_shared<sphere>(point3(-4, 1, 0), 1.0, material2));

	auto material3 = make_shared<metal>(color(0.7, 0.6, 0.5), 0.0);
	world.add(make_shared<sphere>(point3(4, 1, 0), 1.0, material3));

	return world;
}


hittable_list two_perlin_spheres() {
	hittable_list objects;

	auto pertext = make_shared<noise_texture>(4, noise_texture::NoiseType::NT_marbled);
	objects.add(make_shared<sphere>(point3(0, -1000, 0), 1000, make_shared<lambertian>(pertext)));
	objects.add(make_shared<sphere>(point3(0, 2, 0), 2, make_shared<lambertian>(pertext)));

	auto diffuseLight = make_shared<diffuse_light>(color(4, 4, 4));
	auto diffuseLightRed = make_shared<diffuse_light>(color(3.8, 0, 0));
	auto rectLight = make_shared<xy_rect>(3, 5, 1, 3, -2, diffuseLight);
	objects.add(make_shared<sphere>(point3(0, 5, 0), 0.6, diffuseLightRed));
	objects.add(rectLight);
	return objects;
}

hittable_list cornell_box() {
	hittable_list objects;

	auto red = make_shared<lambertian>(color(.65, .05, .05));
	auto white = make_shared<lambertian>(color(.73, .73, .73));
	auto green = make_shared<lambertian>(color(.12, .45, .15));
	auto light = make_shared<diffuse_light>(color(15, 15, 15));

	objects.add(make_shared<yz_rect>(0, 555, 0, 555, 555, green));
	objects.add(make_shared<yz_rect>(0, 555, 0, 555, 0, red));
	auto lightPlane = make_shared<xz_rect>(213, 343, 227, 332, 554, light);
	objects.add(make_shared<flip_face>(lightPlane));
	objects.add(make_shared<xz_rect>(0, 555, 0, 555, 0, white));
	objects.add(make_shared<xz_rect>(0, 555, 0, 555, 555, white));
	objects.add(make_shared<xy_rect>(0, 555, 0, 555, 555, white));

	//objects.add(make_shared<box>(point3(130, 0, 65), point3(295, 165, 230), white));
	//objects.add(make_shared<box>(point3(265, 0, 295), point3(430, 330, 460), white));

	shared_ptr<hittable> box1 = make_shared<box>(point3(0, 0, 0), point3(165, 330, 165), white);
	box1 = make_shared<rotate_y>(box1, 15);
	box1 = make_shared<translate>(box1, vec3(265, 0, 295));
	objects.add(box1);

	shared_ptr<hittable> box2 = make_shared<box>(point3(0, 0, 0), point3(165, 165, 165), white);
	box2 = make_shared<rotate_y>(box2, -18);
	box2 = make_shared<translate>(box2, vec3(130, 0, 65));
	objects.add(box2);

	return objects;
}

hittable_list earth() {
	hittable_list objects;
	//auto groundMatMetal = make_shared<metal>(color(0.6,0.6,0.6),0.4);
	auto groundMatLambert = make_shared<lambertian>(color(0.6, 0.4, 0.4));
	auto ground = make_shared<xz_rect>(-100, 100, -20, 20, -10, groundMatLambert);
	auto earth_texture = make_shared<image_texture>("earthmap.jpg");
	auto earth_surface = make_shared<lambertian>(earth_texture);
	auto globe = make_shared<sphere>(point3(0, 0, 0), 2, earth_surface);

	auto diffuseLight = make_shared<diffuse_light>(color(5, 5, 5));
	auto rectLight = make_shared<xz_rect>(-2, 2, -2, 2, 2.5, diffuseLight);

	objects.add(globe);
	objects.add(ground);
	objects.add(rectLight);
	return objects;
}

hittable_list custom_scene()
{
	hittable_list objects;

	auto checker = make_shared<checker_texture>(color(0.2, 0.2, 0.2), color(0.9, 0.9, 0.9));
	auto checker_material = make_shared<lambertian>(checker);
	auto material_ground = std::make_shared<metal>(color(0.7, 0.7, 0.7), 0.1);

	auto material_lambert = std::make_shared<lambertian>(color(0.0, 0.5, 0.3));

	auto material_lambert1 = std::make_shared<lambertian>(color(0.8, 0.1, 0.1));

	auto material_glass = std::make_shared<dielectric>(1.5);

	auto material_metal = std::make_shared<metal>(color(0.8, 0.6, 0.2), 0);
	auto material_metal1 = std::make_shared<metal>(color(0.0, 0.2, 0.7), 0);
	auto material_zs = std::make_shared<dielectric>(2.4);//zuan shi

	vec3 pCenter = vec3(0, 0, -1);
	vec3 pLeft = vec3(-1, 0, -1);
	vec3 pLeft_back = vec3(-0.6, 0, -1.8);
	vec3 pRight = vec3(1, 0, -1);

	objects.add(std::make_shared<sphere>(vec3(0, -100.5, -1), 100, checker_material));

	objects.add(std::make_shared<sphere>(pCenter, 0.5, material_lambert));

	objects.add(std::make_shared<sphere>(pLeft, 0.5, material_glass));
	objects.add(std::make_shared<sphere>(pLeft, -0.48, material_glass));

	objects.add(std::make_shared<sphere>(pRight, 0.5, material_metal));

	objects.add(std::make_shared<sphere>(vec3(-0.3, -0.25, 0.0), 0.25, material_zs));

	objects.add(std::make_shared<moving_sphere>(pLeft_back, pLeft_back + vec3(0.3, 0, 0), 0, 1, 0.3, material_metal1));

	objects.add(std::make_shared<moving_sphere>(vec3(0.3, -0.34, 0.0), point3(0.3, -0.1, 0.0), 0.0, 1.0, 0.2, material_lambert1));

	objects.add(std::make_shared<sphere>(pLeft + vec3(0, 0, -5.0), 0.5, material_lambert));
	objects.add(std::make_shared<sphere>(pCenter + vec3(0, 0, -5.0), 0.5, material_glass));
	objects.add(std::make_shared<sphere>(pCenter + vec3(0, 0, -5.0), -0.48, material_glass));
	objects.add(std::make_shared<sphere>(pRight + vec3(0, 0, -5.0), 0.5, material_metal1));

	return objects;
}


hittable_list cornell_smoke() {
	hittable_list objects;

	auto red = make_shared<lambertian>(color(.65, .05, .05));
	auto white = make_shared<lambertian>(color(.73, .73, .73));
	auto green = make_shared<lambertian>(color(.12, .45, .15));
	auto purple = make_shared<lambertian>(color(.6, .1, .4));
	auto blue = make_shared<lambertian>(color(.2, .2, .7));
	auto light = make_shared<diffuse_light>(color(7, 7, 7));

	objects.add(make_shared<yz_rect>(0, 555, 0, 555, 555, green));
	objects.add(make_shared<yz_rect>(0, 555, 0, 555, 0, red));
	objects.add(make_shared<xz_rect>(0, 555, 0, 555, 555, white));
	objects.add(make_shared<xz_rect>(0, 555, 0, 555, 0, blue));
	objects.add(make_shared<xy_rect>(0, 555, 0, 555, 555, purple));

	objects.add(make_shared<xz_rect>(113, 443, 127, 432, 554, light));

	shared_ptr<hittable> box1 = make_shared<box>(point3(0, 0, 0), point3(165, 330, 165), white);
	box1 = make_shared<rotate_y>(box1, 15);
	box1 = make_shared<translate>(box1, vec3(265, 0, 295));

	shared_ptr<hittable> box2 = make_shared<box>(point3(0, 0, 0), point3(165, 165, 165), white);
	box2 = make_shared<rotate_y>(box2, -18);
	box2 = make_shared<translate>(box2, vec3(130, 0, 65));

	objects.add(make_shared<constant_medium>(box1, 0.01, color(0, 0, 0)));
	objects.add(make_shared<constant_medium>(box2, 0.01, color(1, 1, 1)));

	return objects;
}

inline double pdf(const vec3& p) {
	return 1 / (2 * pi);
}

int main1() {
	int N = 1000000;
	auto sum = 0.0;
	for (int i = 0; i < N; i++) {
		vec3 d = random_in_normal_half_sphere(vec3(0, 0, 1));
		auto cosine_squared = d.z();
		sum += cosine_squared / pdf(d);
	}
	std::cout << std::fixed << std::setprecision(12);
	std::cout << "I = " << sum / N << '\n';
	std::cout << "4*pi/3=" << 4. * pi / 3. << std::endl;
	getchar();
	return 0;
}

int main() {

	double aspect_ratio;
	int image_width;
	int image_height;
	int samples_per_pixel;
	color background;
	point3 lookfrom;
	point3 lookat;
	vec3 up = vec3(0, 1, 0);
	double vfov;
	double foucsDist = 1.0 /*(eye - lookat).length()*/;
	double aperture = 0.0;

	//world
	hittable_list world;

	//camera 
	world = cornell_box();
	auto lights = make_shared<xz_rect>(213, 343, 227, 332, 554, shared_ptr<material>());
	aspect_ratio = 1.0;
	image_width = 600;
	image_height = static_cast<int>(image_width / aspect_ratio);
	samples_per_pixel = 10;
	background = color(0, 0, 0);
	lookfrom = point3(278, 278, -800);
	lookat = point3(278, 278, 0);
	vfov = 40.0;

	//world = two_perlin_spheres();
	//aspect_ratio = 16.0/9.0;
	//image_width = 400;
	//image_height = image_width / aspect_ratio;
	//samples_per_pixel = 200;
	//background = color(0, 0, 0);
	//lookfrom = point3(20, 3, 8);
	//lookat = point3(0, 2, 0);
	//vfov = 40.0;

	CameraParam cp =
	{
		lookfrom,lookat,up,vfov,aspect_ratio,aperture,foucsDist,0.0,1.0
	};

	camera cam(cp);


	//render
	std::cout << "P3\n" << image_width << " " << image_height << "\n255\n";
	for (int j = image_height - 1; j >= 0; --j)
	{
		std::cerr << "\rScanlines remaining: " << j << ' ' << std::flush;
		for (int i = 0; i < image_width; i++)
		{
			vec3 pixel_color;

			for (int s = 0; s < samples_per_pixel; ++s)
			{
				auto u = float(i + random_double()) / (image_width - 1);
				auto v = float(j + random_double()) / (image_height - 1);
				ray r = cam.get_ray(u, v);
				pixel_color += ray_color(r, world,lights ,background,maxDepth);
			}

			//颜色filter
			pixel_color /= samples_per_pixel;

			//gamma 矫正
			double r = powl(pixel_color.x(), 1.0 / 2.2);
			double g = powl(pixel_color.y(), 1.0 / 2.2);
			double b = powl(pixel_color.z(), 1.0 / 2.2);

			vec3 res_color = vec3(r, g, b);
			write_color(std::cout, res_color);
		}
	}

	std::cerr << "\nDone.\n";

	return 0;
}