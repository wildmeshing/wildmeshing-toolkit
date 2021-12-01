#include "Delaunay.hpp"
#include <Delaunay_psm.h>

#include <mutex>

namespace wmtk
{
	auto delaunay3D(const std::vector<Point3D> &points)
		-> std::pair<std::vector<Point3D>, std::vector<Tetrahedron>>
	{
		static std::once_flag once_flag;
		std::call_once(once_flag, []()
					   { GEO::initialize(); });

		GEO::Delaunay_var engine = GEO::Delaunay::create(3, "BDEL");
		assert(engine);

		// Some settings
		engine->set_reorder(true);
		engine->set_stores_cicl(false);      // Incident tetrahedral list.
		engine->set_stores_neighbors(false); // Vertex neighbors.
		engine->set_refine(false);
		engine->set_keeps_infinite(false);

		// Run!
		geo_assert(points.size() > 0);
		engine->set_vertices(points.size(), points.front().data());

		// Extract output.
		const size_t num_vertices = engine->nb_vertices();
		const size_t num_tets = engine->nb_cells();
		std::vector<Point3D> vertices(num_vertices);
		std::vector<Tetrahedron> tets(num_tets);

		for (size_t i = 0; i < num_vertices; i++)
		{
			vertices[i][0] = engine->vertex_ptr(i)[0];
			vertices[i][1] = engine->vertex_ptr(i)[1];
			vertices[i][2] = engine->vertex_ptr(i)[2];
		}

		for (size_t i = 0; i < num_tets; i++)
		{
			tets[i][0] = engine->cell_vertex(i, 0);
			tets[i][1] = engine->cell_vertex(i, 1);
			tets[i][2] = engine->cell_vertex(i, 2);
			tets[i][3] = engine->cell_vertex(i, 3);
		}

		return {vertices, tets};
	}

} // namespace wmtk
