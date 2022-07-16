// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>


rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f> &cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}


static bool insideTriangle(int x, int y, const Vector3f* _v)
{   
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
    float v0x = _v[0].x(), v0y = _v[0].y(), v0z = _v[0].z();
    float v1x = _v[1].x(), v1y = _v[1].y(), v1z = _v[1].z();
    float v2x = _v[2].x(), v2y = _v[2].y(), v2z = _v[2].z();

    Vector3f v01(v1x - v0x, v1y - v0y, v1z - v0z);
    Vector3f v12(v2x - v1x, v2y - v1y, v2z - v1z);
    Vector3f v20(v0x - v2x, v0y - v2y, v0z - v2z);
    Vector3f v1(x - v0x, y - v0y, 1 - v0z);
    Vector3f v2(x - v1x, y - v1y, 1 - v1z);
    Vector3f v3(x - v2x, y - v2y, 1 - v2z);

    return (v01.cross(v1).z() > 0 && v12.cross(v2).z() > 0 && v20.cross(v3).z() > 0) 
            || (v01.cross(v1).z() < 0 && v12.cross(v2).z() < 0 && v20.cross(v3).z() < 0);
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
{
    auto& buf = pos_buf[pos_buffer.pos_id];
    auto& ind = ind_buf[ind_buffer.ind_id];
    auto& col = col_buf[col_buffer.col_id];

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    for (auto& i : ind)
    {
        Triangle t;
        Eigen::Vector4f v[] = {
                mvp * to_vec4(buf[i[0]], 1.0f),
                mvp * to_vec4(buf[i[1]], 1.0f),
                mvp * to_vec4(buf[i[2]], 1.0f)
        };
        //Homogeneous division
        for (auto& vec : v) {
            vec /= vec.w();
        }
        //Viewport transformation
        for (auto & vert : v)
        {
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
            vert.z() = vert.z() * f1 + f2;
        }

        for (int i = 0; i < 3; ++i)
        {
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
        }

        auto col_x = col[i[0]];
        auto col_y = col[i[1]];
        auto col_z = col[i[2]];

        t.setColor(0, col_x[0], col_x[1], col_x[2]);
        t.setColor(1, col_y[0], col_y[1], col_y[2]);
        t.setColor(2, col_z[0], col_z[1], col_z[2]);

        rasterize_triangle(t);
    }
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    bool MSAA = true;
    auto v = t.toVector4();
    int x_min = floor(std::min(std::min(v[0].x(), v[1].x()), v[2].x()));
    int x_max = ceil(std::max(std::max(v[0].x(), v[1].x()), v[2].x()));
    int y_min = floor(std::min(std::min(v[0].y(), v[1].y()), v[2].y()));
    int y_max = ceil(std::max(std::max(v[0].y(), v[1].y()), v[2].y()));
    if(MSAA) {
        for(int i = x_min; i <= x_max; i++) {
            for(int j = y_min; j <= y_max; j++) {
                // 如果在三角形内部
                float minDepth = FLT_MAX;
                int eid = get_index(i, j) * 4;
                Eigen::Vector3f sum_colour({0, 0, 0});
                std::vector<std::pair<float, float>> pos
                {
                    {2.5, 2.5},
                    {7.5, 2.5},
                    {2.5, 7.5},
                    {7.5, 7.5}
                };
                for(int z = 0; z < 4; z++) {  
                    int x = i + pos[z].first;
                    int y = j + pos[z].second;
                    if(insideTriangle(x, y, t.v)) {
                        auto tuple3f = computeBarycentric2D(x, y, t.v);
                        float alpha = std::get<0>(tuple3f);
                        float beta = std::get<1>(tuple3f);
                        float gamma = std::get<2>(tuple3f);
                        float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                        float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                        z_interpolated *= w_reciprocal;
                        if(depth_sample[eid + z] > z_interpolated) {
                            depth_sample[eid + z] = z_interpolated;
                            frame_sample[eid + z] = t.getColor();
                        }
                        minDepth = std::min(minDepth, depth_sample[eid + z]);
                    }
                }
                Eigen::Vector3f color = (frame_sample[eid] + frame_sample[eid + 1] + frame_sample[eid + 2] + frame_sample[eid + 3])/4;
                set_pixel({i,j,1}, color);
                depth_buf[get_index(i,j)] = std::min(depth_buf[get_index(i,j)], minDepth);  

            }
        }
    }
    else {
        for(int i = x_min; i <= x_max; i++) {
            for(int j = y_min; j <= y_max; j++) {
                if(insideTriangle(i + 0.5, j + 0.5, t.v)) {
                    auto tuple3f = computeBarycentric2D(i + 0.5, j + 0.5, t.v);
                    float alpha = std::get<0>(tuple3f);
                    float beta = std::get<1>(tuple3f);
                    float gamma = std::get<2>(tuple3f);
                    float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                    float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                    z_interpolated *= w_reciprocal;
                    // 深度进行比较
                    if(depth_buf[get_index(i, j)] > z_interpolated) {
                        depth_buf[get_index(i, j)] = z_interpolated;
                        set_pixel(Eigen::Vector3f(static_cast<float>(i), static_cast<float>(j), z_interpolated), 
                                    t.getColor());
                    }
                }
            }
        }
    }
    // TODO : Find out the bounding box of current triangle.
    // iterate through the pixel and find if the current pixel is inside the triangle

    // If so, use the following code to get the interpolated z value.
    

    // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.
}
    
    
    

void rst::rasterizer::set_model(const Eigen::Matrix4f& m)
{
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v)
{
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p)
{
    projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff)
{
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        std::fill(frame_sample.begin(), frame_sample.end(), Eigen::Vector3f{0, 0, 0});
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
        std::fill(depth_sample.begin(), depth_sample.end(), std::numeric_limits<float>::infinity());
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);
    depth_sample.resize(4 * w * h);
    frame_sample.resize(4 * w * h);
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height-1-y)*width + x;
}

int rst::rasterizer::get_sample_index(int x, int y) {
    return (height*2 - 1 - y)*width*2 + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (height-1-point.y())*width + point.x();
    frame_buf[ind] = color;

}

void rst::rasterizer::set_sample_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (2 * height-1-point.y())* 2 *width + point.x();
    frame_sample[ind] = color;
}

// clang-format on