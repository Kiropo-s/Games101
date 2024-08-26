//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"

void Scene::buildBVH()
{
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k)
    {
        if (objects[k]->hasEmit())
        {
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k)
    {
        if (objects[k]->hasEmit())
        {
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum)
            {
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
    const Ray &ray,
    const std::vector<Object *> &objects,
    float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k)
    {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear)
        {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }

    return (*hitObject != nullptr);
}

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // TO DO Implement Path Tracing Algorithm here
    Vector3f L_dir(0, 0, 0), L_indir(0, 0, 0);

    Intersection inter = intersect(ray);

    if (inter.happened)
    {
        if (inter.m->hasEmission())
        {
            L_dir = inter.m->getEmission();
        }
        else
        {
            Vector3f pos = inter.coords;
            Vector3f n = inter.normal;//.normalized();
            Vector3f wo = ray.direction;
            Material *m = inter.m;
            float pdf_ligeht = 0.0;
            Intersection light_inter;

            sampleLight(light_inter, pdf_ligeht);

            Vector3f x = light_inter.coords;
            Vector3f ws = (x - pos).normalized();
            Vector3f NN = light_inter.normal;//.normalized();
            Vector3f emit = light_inter.emit;
            float dis_light = (x - pos).norm();

            Ray ro = Ray(pos, ws);
            if (intersect(ro).distance - dis_light > -0.001)
            {
                L_dir = emit * m->eval(wo, ws, n) * dotProduct(ws, n) * dotProduct(-ws, NN) / (dis_light * dis_light) / pdf_ligeht;
            }

            float P_RR = get_random_float();
            if (P_RR < RussianRoulette)
            {
                Vector3f wi = m->sample(wo, n).normalized();
                Ray r(pos, wi);
                Intersection obj_inter = intersect(r);
                if (obj_inter.happened && !(obj_inter.m->hasEmission()))
                {
                    L_indir = castRay(r, depth + 1) * m->eval(wo, wi, n) * dotProduct(wi, n) / m->pdf(wo, wi, n) / RussianRoulette;
                }
            }
        }
    }

    return L_dir + L_indir;
}