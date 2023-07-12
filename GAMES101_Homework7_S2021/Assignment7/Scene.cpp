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
        if (objects[k]->hasEmit()){
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
    //Intersection inter = intersect(ray);// 获取相交信息
    //if (!inter.happened) return Vector3f();// 没有相交 直接返回
    //if (inter.m->hasEmission()) return inter.m->getEmission();// 打到光源 直接返回光源颜色

    Vector3f dir=(0,0,0), indir=(0,0,0);// 入射光方向与反方向
    //获取相交信息
    Intersection inter = intersect(ray);
    //如果没发生相交，则返回黑色
    if (!inter.happened)
    {
        return dir;
    }
    //如果相交点是光源，则返回材质的自发光项
    if (inter.m->hasEmission())
    {
        //return inter.m->getEmission();
        if (depth == 0) {//第一次打到光
            return inter.m->getEmission();
        }
        else return dir;//弹射打到光，直接返回0，0.0

    }
    //如果相交的是物体,那么我们需要获取物体的部分信息；
    //物体相交的位置
    Vector3f obj_pos = inter.coords;
    //物体的法线,归一
    Vector3f obj_nor = inter.normal.normalized();
    //计算视线到物体
    Vector3f obj_dir = ray.direction;
    //定义光源的一些参数,先对光源的周围进行积分
    Intersection light_point;
    //定义光源pdf
    float light_pdf=0.0f;
    sampleLight(light_point, light_pdf);

    //接下来我们获取光源某一方位角的数据
    //位置
    Vector3f xx = light_point.coords;
    //法线
    Vector3f nn = light_point.normal.normalized();
    // lightto hitpoint
    Vector3f l2o_dir = (obj_pos - xx).normalized();
    //距离
    float distance = (obj_pos - xx).norm();
    //float dis2 = distance * distance;
    float dis2 = dotProduct((obj_pos - xx), (obj_pos - xx));

    //发射光线
    Ray light_to_obj(xx, l2o_dir);
    //光线和物体求交
    Intersection interlight2o = intersect(light_to_obj);
    //如果dis比射线相交时的时候的dis大，则说明中间有物体遮挡
    if (interlight2o.happened && interlight2o.distance - distance > -EPSILON)
    {
        //没有遮挡且碰撞发生时，代入公式
        //光强
        Vector3f l_i = light_point.emit;
        //brdf
        Vector3f f_r = inter.m->eval(obj_dir, -l2o_dir, obj_nor);
        //物体to光线和法线的夹角the1
        float cos_theta1 = dotProduct( -l2o_dir,obj_nor);
        //光源夹角
        float cos_theta2 = dotProduct( l2o_dir,nn);
        dir = l_i * f_r * cos_theta1 * cos_theta2 / dis2 / light_pdf;
    }

    //求间接光照
    //使用俄罗斯转盘赌
    float ksi = get_random_float();
    if (ksi < RussianRoulette)
    {
        //计算间接光照,返回一个随机的方向
        //sample
        //将光源进行按面采样,随机从光源发射一条ray打到场景中的sphere上得到某个交点
        Vector3f wi = inter.m->sample(obj_dir, obj_nor).normalized();
        //创建从物体出发，往四面的射线
        Ray r(obj_pos, wi);
        //求出光线与场景的交点
        Intersection obj2scene = intersect(r);
        //光线与物体相交且不是光源
        if (obj2scene.happened&&!obj2scene.m->hasEmission())
        {
            Vector3f f_r1 = obj2scene.m->eval(obj_dir, wi, obj_nor);
            float cos_the2 = dotProduct(wi, obj_nor);
            float pdf_hemi = inter.m->pdf(obj_dir, wi, obj_nor);
            indir = castRay(r, depth + 1) * f_r1 * cos_the2 / pdf_hemi / RussianRoulette;
        }
    }

    return dir + indir;
}