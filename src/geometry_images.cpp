#include "geometry_images.h"
#include <deque>
#include <igl/edge_topology.h>
#include <igl/is_boundary_edge.h>
#include <igl/vertex_triangle_adjacency.h>
#include <igl/triangle_triangle_adjacency.h>
#include <igl/is_border_vertex.h>
#include <igl/harmonic.h>
#include <igl/cut_mesh.h>
#include <igl/boundary_loop.h>
#include <igl/vertex_components.h>
#include <igl/map_vertices_to_circle.h>
#include <cmath>
#include <unordered_set>

int check_components(const Eigen::MatrixXd & V,
                     const Eigen::MatrixXi & F)
{
    Eigen::VectorXi components;  // per-vertex component IDs
    igl::vertex_components(F, components);

    std::unordered_set<int> unique_component_ids;
    for (int ci = 0; ci < components.size(); ci++)
        unique_component_ids.insert(components[ci]);

    return unique_component_ids.size();
}

// Given a 3D mesh of arbitrary genus,
// find a cut that opens the mesh into a topological disk.
void initial_cut(const Eigen::MatrixXd & V,
                 const Eigen::MatrixXi & F,
                 float seed,
                 Eigen::ArrayXi & cut_edges)
{
    // Edges
    Eigen::MatrixXi E;
    Eigen::MatrixXi FE;
    Eigen::MatrixXi EF;
    igl::edge_topology(V, F, E, FE, EF);
    printf("V: %d | F: %d | E: %d\n", V.rows(), F.rows(), E.rows());

    // Vertex-face topology
    std::vector<std::vector<int>> VF;
    std::vector<std::vector<int>> VFi;
    igl::vertex_triangle_adjacency(V, F, VF, VFi);

    // Detect the boundary edges
    Eigen::ArrayXi boundary_edges;
    igl::is_boundary_edge(E, F, boundary_edges);

    Eigen::ArrayXi remaining_edges = 1 - boundary_edges;
    Eigen::ArrayXi remaining_vertices = Eigen::ArrayXi::Ones(V.rows());
    Eigen::ArrayXi remaining_faces = Eigen::ArrayXi::Ones(F.rows());

    // Remove seed triangle
    std::deque<int> new_boundary_edges;
    int seed_idx = int(seed * ((float) F.rows()));
    remaining_faces[seed_idx] = 0;
    new_boundary_edges.push_back(FE(seed_idx, 0));
    new_boundary_edges.push_back(FE(seed_idx, 1));
    new_boundary_edges.push_back(FE(seed_idx, 2));

    std::deque<int> suspect_vertices;
    auto remove_face = [&](const int & face,
                           const int & old_boundary_edge)
    {
        remaining_faces[face] = 0;
        Eigen::Vector3i face_edges = FE.row(face);
        if (face_edges(0) != old_boundary_edge)
            new_boundary_edges.push_back(face_edges(0));
        if (face_edges(1) != old_boundary_edge)
            new_boundary_edges.push_back(face_edges(1));
        if (face_edges(2) != old_boundary_edge)
            new_boundary_edges.push_back(face_edges(2));

        for (int vi = 0; vi < 3; vi++)
        {
            int vertex = F(face, vi);
            suspect_vertices.push_back(vertex);
        }
    };

    // Iteratively remove newly-created boundary edges
    while (!new_boundary_edges.empty())
    {
        int edge = new_boundary_edges.front();
        new_boundary_edges.pop_front();  // breadth-first

        int face0 = EF(edge, 0);
        int face1 = EF(edge, 1);
        bool face0_remains = face0 != -1 && remaining_faces[face0];
        bool face1_remains = face1 != -1 && remaining_faces[face1];

        if (face0_remains && !face1_remains)
        {
            remaining_edges[edge] = 0;
            remove_face(face0, edge);
        }
        else if (!face0_remains && face1_remains)
        {
            remaining_edges[edge] = 0;
            remove_face(face1, edge);
        }
    }

    // Iteratively remove newly-dangling edges
    while (!suspect_vertices.empty())
    {
        int vertex = suspect_vertices.front();
        suspect_vertices.pop_front();  // breadth-first

        if (vertex != -1 && remaining_vertices[vertex])
        {
            // Check whether vertex is adjacent to only one edge
            int num_adjacent_edges = 0;
            int adjacent_edge = -1;
            int other_vertex = -1;
            for (auto & face : VF[vertex])
            {
                for (int ei = 0; ei < 3; ei++)
                {
                    int edge = FE(face, ei);
                    if (edge != adjacent_edge &&
                        (remaining_edges[edge] || boundary_edges[edge]))
                    {
                        if (E(edge, 0) == vertex)
                        {
                            num_adjacent_edges++;
                            adjacent_edge = edge;
                            other_vertex = E(edge, 1);
                        }
                        else if (E(edge, 1) == vertex)
                        {
                            num_adjacent_edges++;
                            adjacent_edge = edge;
                            other_vertex = E(edge, 0);
                        }
                    }
                }
            }
            if (num_adjacent_edges == 1)
            {
                remaining_vertices[vertex] = 0;
                remaining_edges[adjacent_edge] = 0;
                suspect_vertices.push_back(other_vertex);
            }
        }
    }

    if (remaining_vertices.sum() == 1)
    {
        // Add back two adjacent edges to the remaining vertex
        for (int vertex = 0; vertex < V.rows(); vertex++)
        {
            if (remaining_vertices[vertex])
            {
                bool done = false;
                int num_edges_added = 0;
                int prev_edge_added = -1;
                for (auto & face : VF[vertex])
                {
                    for (int ei = 0; ei < 3; ei++)
                    {
                        int edge = FE(face, ei);
                        if (edge != prev_edge_added &&
                            !remaining_edges[edge] &&
                            (E(edge, 0) == vertex || E(edge, 1) == vertex))
                        {
                            remaining_edges[edge] = 1;
                            num_edges_added++;
                            prev_edge_added = edge;

                            if (num_edges_added >= 2)
                                break;
                        }
                    }
                    if (done)
                        break;
                }
                break;
            }
        }
    }

    // Straighten cut-paths
    // Replace each with constrained shortest path between cut-nodes

    cut_edges = remaining_edges + boundary_edges;  // re-add boundary edges
}

void open_cut(const Eigen::MatrixXd & V,
              const Eigen::MatrixXi & F,
              const Eigen::ArrayXi & cut_edges,
              Eigen::MatrixXd & Vcut,
              Eigen::MatrixXi & Fcut,
              Eigen::VectorXi & boundary)
{
    // Edges
    Eigen::MatrixXi E;
    Eigen::MatrixXi FE;
    Eigen::MatrixXi EF;
    igl::edge_topology(V, F, E, FE, EF);

    // Vertex-face topology
    std::vector<std::vector<int>> VF;
    std::vector<std::vector<int>> VFi;
    igl::vertex_triangle_adjacency(V, F, VF, VFi);

    // Face-face topology
    Eigen::MatrixXi TT;
    Eigen::MatrixXi TTi;
    igl::triangle_triangle_adjacency(F, TT, TTi);

    // Border vertex identification
    std::vector<bool> V_border = igl::is_border_vertex(V, F);

    // Define edges to cut
    Eigen::MatrixXi cuts = Eigen::MatrixXi::Zero(F.rows(), 3);
    for (int edge = 0; edge < cut_edges.size(); edge++)
    {
        if (cut_edges[edge])
        {
            for (int fi = 0; fi < 2; fi++)
            {
                int face = EF(edge, fi);
                if (face != -1)
                {
                    int ei = 0;
                    while (FE(face, ei) != edge) ei++;
                    cuts(face, ei) = 1;
                }
            }
        }
    }

    // Open mesh
    igl::cut_mesh(V, F, VF, VFi, TT, TTi, V_border, cuts, Vcut, Fcut);

    // Get boundary indices into Vcut
    igl::boundary_loop(Fcut, boundary);
}

// Map opened cut nodes to grid points on the boundary of the unit square.
void boundary_parameterization(const Eigen::MatrixXd & V,
                               const Eigen::MatrixXi & F,
                               const Eigen::VectorXi & boundary,
                               Eigen::MatrixXd & boundary_uv)
{
    // Output N x 2 list of 2D positions on the unit square boundary
    int n = boundary.size();
    boundary_uv = Eigen::MatrixXd::Zero(n, 2);
    double increment = 5.0 / static_cast<double>(n);
    double i = 0;
    for (int vi = 0; vi < n; vi++)
    {
        switch(int(std::floor(i)))
        {
            case 0:
                boundary_uv(vi, 0) = 1.0;
                boundary_uv(vi, 1) = i;
                break;
            case 1:
                boundary_uv(vi, 0) = 1.0 - (i - 1.0) * 2.0;
                boundary_uv(vi, 1) = 1.0;
                break;
            case 2:
                boundary_uv(vi, 0) = -1.0;
                boundary_uv(vi, 1) = 1.0 - (i - 2.0) * 2.0;
                break;
            case 3:
                boundary_uv(vi, 0) = -1.0 + (i - 3.0) * 2.0;
                boundary_uv(vi, 1) = -1.0;
                break;
            case 4:
                boundary_uv(vi, 0) = 1.0;
                boundary_uv(vi, 1) = -1.0 + (i - 4.0);
                break;
        }
        i += increment;
    }
}

void geometry_image(const Eigen::MatrixXd & V,
                    const Eigen::MatrixXi & F,
                    Eigen::MatrixXd & U,
                    Eigen::MatrixXd & Vcut,
                    Eigen::MatrixXi & Fcut)
{
    int num_components = check_components(V, F);
    printf("This mesh has %d connected component(s)\n", num_components);
    if (num_components != 1)
        printf("[-] Warning: number of connected components should be 1\n", num_components);

    // Split into connected components
    // Process each, allocate rectangular space in the output UV
    // parameterization proportional to number of vertices in the component

    // Convert to topological disk, define boundary
    bool single_boundary_loop = false;
    Eigen::VectorXi boundary;
    if (single_boundary_loop)
    {
        igl::boundary_loop(F, boundary);
        Vcut = V;
        Fcut = F;
    }
    else
    {
        Eigen::ArrayXi cut_edges;
        initial_cut(V, F, 0.59f, cut_edges);
        printf("Number of edges in initial cut: %d\n", cut_edges.sum());
        open_cut(V, F, cut_edges, Vcut, Fcut, boundary);
        printf("Finished opening cut.\n");
    }

    // Parameterize boundary
    bool circle_boundary = false;
    Eigen::MatrixXd boundary_uv;
    if (circle_boundary)
        igl::map_vertices_to_circle(Vcut, boundary, boundary_uv);
    else
        boundary_parameterization(Vcut, Fcut, boundary, boundary_uv);
    printf("Finished parameterizing boundary.\n");

    // Parameterize interior
    igl::harmonic(Vcut, Fcut, boundary, boundary_uv, 1, U);
    printf("Finished parameterizing interior.\n");
}
