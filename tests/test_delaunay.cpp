#include <wmtk/Delaunay.hpp>

#include <catch2/catch.hpp>

#include <limits>

namespace
{
	inline double compute_tet_volume(
		const wmtk::Point3D &p,
		const wmtk::Point3D &q,
		const wmtk::Point3D &r,
		const wmtk::Point3D &s)
	{
		return -(
			p[0] * q[1] * r[2] - p[0] * q[1] * s[2] - p[0] * q[2] * r[1] + p[0] * q[2] * s[1] + p[0] * r[1] * s[2] - p[0] * r[2] * s[1] - p[1] * q[0] * r[2] + p[1] * q[0] * s[2] + p[1] * q[2] * r[0] - p[1] * q[2] * s[0] - p[1] * r[0] * s[2] + p[1] * r[2] * s[0] + p[2] * q[0] * r[1] - p[2] * q[0] * s[1] - p[2] * q[1] * r[0] + p[2] * q[1] * s[0] + p[2] * r[0] * s[1] - p[2] * r[1] * s[0] - q[0] * r[1] * s[2] + q[0] * r[2] * s[1] + q[1] * r[0] * s[2] - q[1] * r[2] * s[0] - q[2] * r[0] * s[1] + q[2] * r[1] * s[0]);
	}
} // namespace

TEST_CASE("Delaunay3D", "[delaunay][3d]")
{
	using namespace wmtk;

	auto validate = [](const auto &vertices, const auto &tets)
	{
		constexpr double EPS = std::numeric_limits<double>::epsilon();
		const size_t num_vertices = vertices.size();
		const size_t num_tets = tets.size();

		for (size_t i = 0; i < num_tets; i++)
		{
			const auto &tet = tets[i];
			REQUIRE(tet[0] < num_vertices);
			REQUIRE(tet[1] < num_vertices);
			REQUIRE(tet[2] < num_vertices);
			REQUIRE(tet[3] < num_vertices);

			// Tet must be positive oriented and non-degenerate.
			REQUIRE(compute_tet_volume(
						vertices[tet[0]],
						vertices[tet[1]],
						vertices[tet[2]],
						vertices[tet[3]])
					> EPS);
		}
	};

	SECTION("Simple")
	{
		std::vector<Point3D> points{
            {{0, 0, 0}},
			{{1, 0, 0}},
			{{0, 1, 0}},
			{{0, 0, 1}}};

		auto [vertices, tets] = delaunay3D(points);
		REQUIRE(vertices.size() == 4);
		REQUIRE(tets.size() == 1);
		validate(vertices, tets);
	}
	SECTION("Insufficient points should not fail")
	{
		std::vector<Point3D> points{
            {{1, 0, 0}},
			{{0, 1, 0}},
			{{0, 0, 1}}};

		auto [vertices, tets] = delaunay3D(points);
		REQUIRE(tets.size() == 0);
		validate(vertices, tets);
	}
	SECTION("Coplanar pts")
	{
		std::vector<Point3D> points{
            {{0, 0, 0}},
			{{1, 0, 0}},
			{{0, 1, 0}},
			{{0.5, 0.5, 0}}};

		auto [vertices, tets] = delaunay3D(points);
		REQUIRE(vertices.size() == 4);
		REQUIRE(tets.size() == 0);
		validate(vertices, tets);
	}
	SECTION("Duplicate pts")
	{
		std::vector<Point3D> points{
            {{0, 0, 0}},
			{{1, 0, 0}},
			{{0, 1, 0}},
			{{0, 0, 1}},
			{{0, 1, 0}},
			{{0, 1, 0}}};

		auto [vertices, tets] = delaunay3D(points);
		REQUIRE(vertices.size() == 6); // duplicate pts are kept in the output.
		REQUIRE(tets.size() == 1);
		validate(vertices, tets);
	}
	SECTION("Triangle prism")
	{
		std::vector<Point3D> points{
            {{0, 0, 0}},
			{{1, 0, 0}},
			{{0, 1, 0}},
			{{0, 0, 1}},
			{{1, 0, 1}},
			{{0, 1, 1}}};

		auto [vertices, tets] = delaunay3D(points);
		REQUIRE(vertices.size() == 6);
		REQUIRE(tets.size() == 3);
		validate(vertices, tets);
	}
	SECTION("Cube with centroid")
	{
		std::vector<Point3D> points{
            {{0, 0, 0}},
			{{1, 0, 0}},
			{{1, 1, 0}},
			{{0, 1, 0}},
			{{0, 0, 1}},
			{{1, 0, 1}},
			{{1, 1, 1}},
			{{0, 1, 1}},
			{{0.5, 0.5, 0.5}}};

		auto [vertices, tets] = delaunay3D(points);
		REQUIRE(vertices.size() == 9);
		REQUIRE(tets.size() == 12);
		validate(vertices, tets);
	}
	SECTION("Cube with face center")
	{
		std::vector<Point3D> points{
            {{0, 0, 0}},
			{{1, 0, 0}},
			{{1, 1, 0}},
			{{0, 1, 0}},
			{{0, 0, 1}},
			{{1, 0, 1}},
			{{1, 1, 1}},
			{{0, 1, 1}},
			{{0.5, 0.5, 0}}};

		auto [vertices, tets] = delaunay3D(points);
		REQUIRE(vertices.size() == 9);
		REQUIRE(tets.size() == 10);
		validate(vertices, tets);
	}
	SECTION("Cube with point near face")
	{
		std::vector<Point3D> points{
            {{0, 0, 0}},
			{{1, 0, 0}},
			{{1, 1, 0}},
			{{0, 1, 0}},
			{{0, 0, 1}},
			{{1, 0, 1}},
			{{1, 1, 1}},
			{{0, 1, 1}},
			{{0.5, 0.5, 1e-12}}};

		auto [vertices, tets] = delaunay3D(points);
		REQUIRE(vertices.size() == 9);
		REQUIRE(tets.size() == 12);
		validate(vertices, tets);
	}
	SECTION("Regular grid")
	{
		constexpr size_t N = 10;
		std::vector<Point3D> points;
		points.reserve(N * N * N);
		for (size_t i = 0; i < N; i++)
		{
			for (size_t j = 0; j < N; j++)
			{
				for (size_t k = 0; k < N; k++)
				{
					double x = i;
					double y = j;
					double z = k;
					points.push_back({x, y, z});
				}
			}
		}
		auto [vertices, tets] = delaunay3D(points);
		REQUIRE(vertices.size() == N * N * N);
		REQUIRE(tets.size() == (N - 1) * (N - 1) * (N - 1) * 6);
		validate(vertices, tets);
	}
}
