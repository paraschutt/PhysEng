#include <VHACD.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <cstring>

// Matches runtime header structure
#pragma pack(push, 1)
struct APCHeader {
    char magic[4];       // 'APCC'
    uint16_t version;
    uint16_t hull_count;
};
struct APCVertex { float x, y, z; };
struct APCTriangle { uint32_t a, b, c; };
#pragma pack(pop)

struct Vec3 { float x, y, z; };

// Tiny OBJ parser stub (using standard library for demo, production uses tinyobj)
bool load_obj(const char* path, std::vector<Vec3>& out_vertices, std::vector<uint32_t>& out_indices) {
    // ... standard ifstream parsing ...
    // Assume populated for brevity
    return true;
}

int main(int argc, char* argv[]) {
    if (argc != 3) {
        std::cerr << "Usage: vhacd_cli <input.obj> <output.apccol>\n";
        return 1;
    }

    std::vector<Vec3> vertices;
    std::vector<uint32_t> indices;
    if (!load_obj(argv[1], vertices, indices)) return 2;

    // Run V-HACD
    VHACD::IVHACD::Parameters params;
    params.m_maxHullVertices = 32;   // Keep GJK fast
    params.m_resolution = 64;        // Decimation resolution
    params.m_concavity = 0.0025;     // Aggressive merging for performance
    params.m_planeDownsampling = 4;
    params.m_convexhullDownsampling = 4;
    params.m_maxNumVerticesPerCH = 32;
    
    VHACD::IVHACD* interface = VHACD::CreateVHACD();
    bool res = interface->Compute(
        (const double*)vertices.data(), vertices.size(), 
        (const uint32_t*)indices.data(), indices.size() / 3, 
        params
    );

    if (!res) {
        std::cerr << "V-HACD computation failed.\n";
        VHACD::Destroy(interface);
        return 3;
    }

    std::ofstream out_file(argv[2], std::ios::binary);
    
    APCHeader header;
    memcpy(header.magic, "APCC", 4);
    header.version = 1;
    header.hull_count = interface->GetNConvexHulls();
    out_file.write(reinterpret_cast<const char*>(&header), sizeof(header));

    // Extract and write hulls
    for (uint32_t i = 0; i < header.hull_count; ++i) {
        VHACD::IVHACD::ConvexHull hull;
        interface->GetConvexHull(i, hull);

        uint32_t vert_count = hull.m_nPoints;
        uint32_t tri_count = hull.m_nTriangles;
        
        out_file.write(reinterpret_cast<const char*>(&vert_count), sizeof(vert_count));
        out_file.write(reinterpret_cast<const char*>(&tri_count), sizeof(tri_count));

        // Write vertices (double -> float conversion)
        for (uint32_t v = 0; v < vert_count; ++v) {
            APCVertex av;
            av.x = static_cast<float>(hull.m_points[v * 3]);
            av.y = static_cast<float>(hull.m_points[v * 3 + 1]);
            av.z = static_cast<float>(hull.m_points[v * 3 + 2]);
            out_file.write(reinterpret_cast<const char*>(&av), sizeof(av));
        }

        // Write triangles
        for (uint32_t t = 0; t < tri_count; ++t) {
            APCTriangle at;
            at.a = hull.m_triangles[t * 3];
            at.b = hull.m_triangles[t * 3 + 1];
            at.c = hull.m_triangles[t * 3 + 2];
            out_file.write(reinterpret_cast<const char*>(&at), sizeof(at));
        }
    }

    out_file.close();
    VHACD::Destroy(interface);
    
    std::cout << "Successfully exported " << (int)header.hull_count << " hulls to " << argv[2] << "\n";
    return 0;
}