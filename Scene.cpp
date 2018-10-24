// Scene.cpp: implementation.

#include "Scene.h"
#include <stdio.h>
#include <iostream>
#include <iomanip>

namespace ComputerGraphicsCourse {
    Image *RayTrace(const Scene &scene, const Camera &camera) {
        const int width = camera.Width, height = camera.Height;
        Image *image = new Image(width, height);
        int count = 0;

        for (int j = 0; j < height; j++) {
#ifdef _OPENMP
#pragma omp parallel for
#endif
            for (int i = 0; i < width; i++) {
                // YOUR CODE FOR ASSIGNMENT 0 HERE.

                float b = 0.3 * sin(i * M_PI / camera.Height);
                double k = 0.1 * cos(j * M_PI / camera.Height);
                image->Data[width * j + i] = Eigen::Vector3d(0.3 + k, 0.6 + b, 0.7 + k + b); /* Background Color */

                Ray ray = RayThruPixel(camera, i, j);
                IntersectionInfo hit = Intersect(ray, scene);
                if (hit.HitObject != NULL) {
                    image->Data[width * j + i] = FindColor(scene, hit, scene.MaxDepth);
                }
            }
            std::cout << "\rRender: " << std::setprecision(2) << std::fixed << (100.0 * (count++)) / (height - 1) << "%"
                      << std::flush;
        }
        return image;
    }

    Ray RayThruPixel(const Camera &camera, const int i, const int j) {

        // YOUR CODE FOR ASSIGNMENT 2 HERE.
        // Set the origin of the ray p0
        Eigen::Vector3d p0 = camera.LookFrom;
        // set the coordinate frame
        Eigen::Vector3d w = (camera.LookFrom - camera.LookAt).normalized();
        Eigen::Vector3d u = camera.Up.normalized().cross(w).normalized();
        Eigen::Vector3d v = u.cross(-w);

        // Calculate alpha and beta
        double alpha = tan(camera.FoV_X / 2) * ((double(i) - double(camera.Width) / 2) / (double(camera.Width) / 2));
        double beta = tan(camera.FoV_Y / 2) * ((double(camera.Height) / 2 - double(j)) / (double(camera.Height) / 2));

        //direction pi
        Eigen::Vector3d p1 = -w + alpha * u + beta * v;

        return Ray(p0, p1);
    }

    IntersectionInfo Intersect(const Ray &ray, const Scene &scene) {
        double mindist = 1e8;
        Object *hitobject = NULL;
        Eigen::Vector3d hitpos(0, 0, 0);    // hit position
        Eigen::Vector3d normal(0, 0, 0);    // hit position normal
        for (std::vector<Object *>::const_iterator o = scene.objList.begin();
             o < scene.objList.end(); o++) // find closest intersection; test all objects
        {
            Eigen::Vector3d p, n;
            double t = Intersect(ray, *o, p, n);
            if (t > 0 && t < mindist) // closer than previous closest object
            {
                mindist = t;
                hitobject = *o;
                hitpos = p;
                normal = n;
            }
        }

        return IntersectionInfo(hitobject, mindist, hitpos, normal, ray);    // may already be in Intersect()
    }


    double Intersect(const Ray &ray, const Object *obj, Eigen::Vector3d &position, Eigen::Vector3d &normal) {
        Eigen::Vector4d p0(ray.P0[0], ray.P0[1], ray.P0[2], 1), p1(ray.P1[0], ray.P1[1], ray.P1[2], 0);
        // invert transform
        // YOUR CODE FOR ASSIGNMENT 4 HERE.

        Ray transformedRay(p0.block<3, 1>(0, 0), p1.block<3, 1>(0, 0));
        double t = obj->Intersect(transformedRay, position, normal);

        if (t < 0) return t;

        // transform the results
        // YOUR CODE FOR ASSIGNMENT 4 HERE.

        return t;
    }


    double Triangle::Intersect(const Ray &ray, Eigen::Vector3d &position, Eigen::Vector3d &normal) const {
        // YOUR CODE FOR ASSIGNMENT 1 HERE.
        /*TODO: Implement ray-triangle intersection.*/

        // 1. Store 3D position and normal vector (法線ベクトル) of the hit point into position and normal.
        //set parameter
        Eigen::Vector3d a = *(this->vertices[0]);
        Eigen::Vector3d b = *(this->vertices[1]);
        Eigen::Vector3d c = *(this->vertices[2]);
        Eigen::Vector3d nor = this->n;
        Eigen::Vector3d p0 = ray.P0;
        Eigen::Vector3d p1 = ray.P1;
        //set vector of triangle
        Eigen::Vector3d a_b = b - a;
        Eigen::Vector3d c_a = a - c;
        Eigen::Vector3d b_c = c - b;
        Eigen::Vector3d b_a = b - c;
        if (p1.dot(n) == 0)
            return -1;
        double t = (a.dot(n) - p0.dot(n)) / p1.dot(n);
        if (t < 0)
            return -1;
        //store position and normal vector
        position = p0 + t * p1;
        normal = n;
        //set vector of vertex to p
        Eigen::Vector3d a_p = position - a;
        Eigen::Vector3d b_p = position - b;
        Eigen::Vector3d c_p = position - c;



        // edge 0
        Eigen::Vector3d edge0 = a_b.cross(a_p);
        // P is on the right side
        if (nor.dot(edge0) < 0) return -1;

        // edge 1
        Eigen::Vector3d edge1 = b_c.cross(b_p);
        // P is on the right side
        if (nor.dot(edge1) < 0) return -1;

        // edge 2
        Eigen::Vector3d edge2 = c_a.cross(c_p);
        // P is on the right side
        if (nor.dot(edge2) < 0) return -1;
        return t;


    }

    double Sphere::Intersect(const Ray &ray, Eigen::Vector3d &position, Eigen::Vector3d &normal) const {
        // YOUR CODE FOR ASSIGNMENT 2 HERE.
        /*TODO: Implement ray-sphere intersection. */
        /* return positive distance from ray origin to intersection */
        /* return -1, if no sphere intersects */

        //set quadratic equation//
        double a = ray.P1.dot(ray.P1);
        double b = 2 * ray.P1.dot(ray.P0 - this->C);
        double c = (ray.P0 - this->C).dot(ray.P0 - this->C) - this->Radius * this->Radius;
        double delta = b * b - 4 * a * c;

        //no intersection //
        if (delta < 0) {
            return -1;
        } else {
            // 2 roots found //
            double t1 = (-b - sqrt(delta)) / (2 * a);
            double t2 = (-b + sqrt(delta)) / (2 * a);

            // ray though out sphere pick smaller one //
            if (t1 >= 0 && t2 >= 0) {
                position = ray.P0 + t1 * ray.P1;
                normal = (position - this->C).normalized();
                return t1;
            }

                // ray origin inside sphere pick positive one //
            else if ((t1 > 0) != (t2 > 0)) {
                if (t1 > 0) {
                    position = ray.P0 + t1 * ray.P1;
                    normal = (position - this->C).normalized();
                    return t1;
                } else {
                    position = ray.P0 + t2 * ray.P1;
                    normal = (position - this->C).normalized();
                    return t2;
                }
            }

                // no intersection //
            else if (t1 < 0 && t2 < 0) {
                return -1;
            }
        }
    }

#ifdef TRIANGLE_NORMAL
    double TriangleNormal::Intersect(const Ray& ray, Eigen::Vector3d &position, Eigen::Vector3d &normal) const
    {
        /*TODO: Implement ray-triangle intersection. */
        // 1. Ray-triangle intersection

        // 2. interpolate normal by coefficient


        return -1;
    }
#endif

    Eigen::Vector3d FindColor(const Scene &scene, const IntersectionInfo &hit, const int depth) {
        if (hit.HitObject == NULL)
            return Eigen::Vector3d(0, 0, 0);
        Material mtrl(hit.HitObject->material);
        Eigen::Vector3d color = mtrl.ambient + mtrl.emission;

        /* Ignore Ka, Ke, and Ks terms from the shading equation on pp.7 */
        Eigen::Vector3d p = hit.pos;
        Eigen::Vector3d n = hit.nor.normalized();

        // YOUR CODE FOR ASSIGNMENT 3 HERE.

        // traverse each light
        for (int i = 0; i < scene.lights.size(); i++) {
            auto light = scene.lights[i];
            // point light case
            if (light.position[3] == 0) {
                // directional light case
                // initialize light
                Eigen::Vector3d ray_source(light.position[0], light.position[1], light.position[2]);
                Eigen::Vector3d ray_direction(p - ray_source);
                Ray hit_2_source(p - 1e-5 * ray_direction, -ray_direction);
                if (ray_direction.normalized().dot(n) < 0) {
                    // if there is occlusion, shadow happens.
                    bool shadow = false;
                    for (int i = 0; i < scene.objList.size(); i++) {
                        auto object = scene.objList[i];
                        Eigen::Vector3d position(0, 0, 0);
                        Eigen::Vector3d normal(0, 0, 0);
                        if (object->Intersect(hit_2_source, position, normal) != -1 &&
                            (p - position).dot(ray_direction) > 0) {
                            shadow = true;
                            break;
                        }


                    }

                    // If no object is in front of the hit point, there is no shadow
                    if (!shadow) {
                        // 3. calculate diffuse and specular shading
                        // Diffuse reflection
                        color = color +
                                (ray_direction.normalized().dot(n) < 0 ? -ray_direction.normalized().dot(n) : 0) *
                                Eigen::Vector3d(mtrl.diffuse(0) * light.color(0),
                                                mtrl.diffuse(1) * light.color(1),
                                                mtrl.diffuse(2) * light.color(2));

                        // Specular reflection
                        Eigen::Vector3d h = ((hit.ray.P0 - p).normalized() +
                                             (-ray_direction).normalized()).normalized();
                        color = color + Eigen::Vector3d(mtrl.specular(0) * light.color(0),
                                                        mtrl.specular(1) * light.color(1),
                                                        mtrl.specular(2) * light.color(2)) *
                                        (h.dot(n) > 0. ? h.dot(n) : 0.);
                    }
                }
            } else {
                // initialize ray
                Eigen::Vector3d ray_source(light.position[0], light.position[1], light.position[2]);
                Eigen::Vector3d ray_direction(p - ray_source);
                Ray hit_source(p - 1e-5 * ray_direction, -ray_direction);
                // distance from source to hit point
                double t = ray_direction.norm();
                // attenuation
                double attenuation = (1) / (light.attenuation(0) + light.attenuation(1) * t +
                                            light.attenuation(2) * pow(t, 2));
                // if there is occlusion, shadow happens.
                bool Shadow = false;
                for (int i = 0; i < scene.objList.size(); i++) {
                    auto object = scene.objList[i];
                    auto position = new Eigen::Vector3d(0, 0, 0);
                    auto normal = new Eigen::Vector3d(0, 0, 0);
                    double source_object = object->Intersect(hit_source, *position, *normal);
                    // If this intersection point is closer to the light source, there is occlusion
                    if (source_object != -1) {
                        if (object != hit.HitObject) {
                            if (source_object < t) {
                                Shadow = true;
                                break;
                            }
                        }
                    }
                }

                // If no object is in front of the hit point, there is no shadow
                if (!Shadow) {
                    // 3. calculate diffuse and specular shading
                    // Diffuse reflection
                    color = color + Eigen::Vector3d(mtrl.diffuse(0) * light.color(0),
                                                    mtrl.diffuse(1) * light.color(1),
                                                    mtrl.diffuse(2) * light.color(2)) * attenuation *
                                    (ray_direction.normalized().dot(n) < 0. ? -ray_direction.normalized().dot(n)
                                                                            : 0.);
                    // Specular reflection
                    Eigen::Vector3d h = ((hit.ray.P0 - p).normalized() + (ray_source - p).normalized()).normalized();
                    color = color + Eigen::Vector3d(mtrl.specular(0) * light.color(0),
                                                    mtrl.specular(1) * light.color(1),
                                                    mtrl.specular(2) * light.color(2)) * attenuation *
                                    (h.dot(n) > 0. ? h.dot(n) : 0.);
                }
            }
        }
        // 4. *option* calculate recursive specular reflection
        if (depth > 0) {
            // Part of the reflected ray(the part orthogonal to normal vector) is the same as hit.ray.P1
            // The other part is opposite
            // Assuming (hit.ray.P1 + k * normal) dot normal = 0
            // Then k = - (hit.ray.P1 dot normal) / (normal dot normal)
            // The reflected p1 = hit.ray.P1 + 2 * k * normal vector
            Eigen::Vector3d reflected_p1 = hit.ray.P1 - 2 * (hit.ray.P1.dot(n) / n.dot(n)) * n;
            Ray ray_reflect(p + 1e-5 * reflected_p1, reflected_p1);
            // We need to calculate whether there's another hit for the reflected ray
            IntersectionInfo new_hit = Intersect(ray_reflect, scene);
            if (new_hit.HitObject == NULL || new_hit.HitObject == hit.HitObject) {
                return color;
            }
            Eigen::Vector3d reflect_color = FindColor(scene, new_hit, depth - 1);
            return color + Eigen::Vector3d(new_hit.HitObject->material.specular(0) * reflect_color(0),
                                           new_hit.HitObject->material.specular(1) * reflect_color(1),
                                           new_hit.HitObject->material.specular(2) * reflect_color(2));
        }
        return color;

    }

    // Helper rotation function.  Please implement this.
    Eigen::Matrix4d Transform::rotate(const float degrees, const Eigen::Vector3d &axis) {
        Eigen::Matrix4d ret = Eigen::Matrix4d::Identity();
        // YOUR CODE FOR ASSIGNMENT 4 HERE.

        return ret;
    }

    Eigen::Matrix4d Transform::scale(const float &sx, const float &sy, const float &sz) {
        Eigen::Matrix4d ret = Eigen::Matrix4d::Identity();
        // YOUR CODE FOR ASSIGNMENT 4 HERE.

        return ret;
    }

    Eigen::Matrix4d Transform::translate(const float &tx, const float &ty, const float &tz) {
        Eigen::Matrix4d ret = Eigen::Matrix4d::Identity();
        // YOUR CODE FOR ASSIGNMENT 4 HERE.

        return ret;
    }

    Eigen::Vector3d Light::Ambient = Eigen::Vector3d(0.2, 0.2, 0.2);
    Eigen::Vector3d Light::Attenuation = Eigen::Vector3d(1., 0., 0.);

    Eigen::Vector3d Material::Diffuse = Eigen::Vector3d(0., 0., 0.);
    Eigen::Vector3d Material::Emission = Eigen::Vector3d(0., 0., 0.);
    Eigen::Vector3d Material::Specular = Eigen::Vector3d(0., 0., 0.);
    float Material::Shininess = 0.f;

}