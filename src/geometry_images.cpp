#include "geometry_images.h"
#include <deque>
#include <igl/edge_topology.h>
#include <igl/is_boundary_edge.h>
#include <igl/vertex_triangle_adjacency.h>

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
                    if (edge != adjacent_edge && remaining_edges[edge])
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
            else if (num_adjacent_edges == 0)
                remaining_vertices[vertex] = 0;  // delete orphan vertices
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

    cut_edges = remaining_edges;
}

void geometry_image(const Eigen::MatrixXd & V,
                    const Eigen::MatrixXi & F,
                    Eigen::MatrixXd & U)
{
    Eigen::ArrayXi cut_edges;
    initial_cut(V, F, 0.5f, cut_edges);
    printf("Number of edges in initial cut: %d\n", cut_edges.sum());

    // Replace with your code
    U = V.leftCols(2);
}
