//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


void Scene::buildBVH() {
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
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){//can faguang
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
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
    Vector3f result(0);
    Vector3f dir_ray_temp(-ray.direction.x,-ray.direction.y,-ray.direction.z);
    // Ray ray_temp(ray.origin,dir_ray_temp);
    Intersection inter=intersect(ray);
    Vector3f hitpoint=inter.coords;  
    if(inter.happened)
    {
        // std::cout<<1;
        //contribution from light_source 
        Intersection sample_light;
        float p;
        sampleLight(sample_light,p);
        auto light_point=sample_light.coords;
        Vector3f light2p=hitpoint-light_point;
        Vector3f dir_light2p=(light2p).normalized();
        Ray ray_lignt2p(light_point,dir_light2p);
        Intersection inter_light2p=intersect(ray_lignt2p);
        if(inter.obj->hasEmit())
            result+=sample_light.emit;
        else{
        if(inter_light2p.happened &&abs(inter_light2p.distance-light2p.norm())<1e-1)
        {
            // std::cout<<2;
            // std::cout<<inter_light2p.distance<<'\t'<<light2p.norm()<<'\n';

            // std::cout<<dotProduct(sample_light.normal,dir_light2p)<<'\n';
            // if(dotProduct(sample_light.normal,dir_light2p)<0)
            // {
                
            //     std::cout<<sample_light.normal<<'\n';
            //     std::cout<<dir_light2p<<'\n';
            //     std::cout<<hitpoint<<'\n';
            //     std::cout<<light_point<<'\n';
            //     std::cout<<dotProduct(sample_light.normal,dir_light2p)<<'\n';
            // }
            // std::cout<<-dotProduct(inter.normal,dir_light2p)<<'\n';
            // auto temp=sample_light.emit/(4*M_PI*light2p.norm())*inter.m->eval(dir_light2p,dir_ray_temp,inter.normal)*(-dotProduct(inter.normal,dir_light2p))*dotProduct(sample_light.normal,dir_light2p)/light2p.norm()/p;
            auto temp=sample_light.emit*inter.m->eval(dir_light2p,dir_ray_temp,inter.normal)*(-dotProduct(inter.normal,dir_light2p))*dotProduct(sample_light.normal,dir_light2p)/pow(light2p.norm(),2)/p;

            // std::cout<<temp<<'\n'<<'\n';
            if(temp.x>=0&&temp.y>=0&&temp.z>=0)
            result+=temp;
        }

        //contribution from other reflectores
        if(get_random_float()<RussianRoulette)
        {
            Vector3f wi=inter.m->sample(ray.direction,inter.normal);
            Ray ray_p2other(hitpoint,wi);
            Intersection inter_p2other=intersect(ray_p2other);
            if(inter_p2other.happened&&!inter_p2other.obj->hasEmit())
            {
                result+=castRay(ray_p2other,depth+1)*inter.m->eval(wi,dir_ray_temp,inter.normal)*dotProduct(inter.normal,wi)*2*M_PI/RussianRoulette;
            }
        }
        }
    }
    return result;
}