#include <map>
#include <set>
#include <algorithm>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

//#include "../CDT/include/CDT.h"
//#include "../CDT/extras/VerifyTopology.h"

#define CGAL_MESH_2_OPTIMIZER_VERBOSE
//#define CGAL_MESH_2_OPTIMIZERS_DEBUG
//#define CGAL_MESH_2_SIZING_FIELD_USE_BARYCENTRIC_COORDINATES
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Delaunay_mesher_2.h>
#include <CGAL/Delaunay_mesh_face_base_2.h>
#include <CGAL/Delaunay_mesh_vertex_base_2.h>
#include <CGAL/Delaunay_mesh_size_criteria_2.h>

#include "MeshObject.h"

#include "..\server.h"

#include <fstream>

typedef CGAL::Exact_predicates_inexact_constructions_kernel           CGAL_K;
typedef CGAL::Delaunay_mesh_vertex_base_2<CGAL_K>                     CGAL_Vb;
typedef CGAL::Delaunay_mesh_face_base_2<CGAL_K>                       CGAL_Fb;
typedef CGAL::Triangulation_data_structure_2<CGAL_Vb, CGAL_Fb>        CGAL_Tds;
typedef CGAL::Constrained_Delaunay_triangulation_2<CGAL_K, CGAL_Tds>  CGAL_CDT;
typedef CGAL::Delaunay_mesh_size_criteria_2<CGAL_CDT>                 CGAL_Criteria;

typedef CGAL_CDT::Vertex_handle CGAL_Vertex_handle;
typedef CGAL_CDT::Point CGAL_Point;

struct OpenMesh::VertexHandle const OpenMesh::PolyConnectivity::InvalidVertexHandle;

template<class T>
constexpr const T& clamp(const T& v, const T& lo, const T& hi)
{
	assert(!(hi < lo));
	return (v < lo) ? lo : (hi < v) ? hi : v;
}

inline float cot(MyMesh::Point v, MyMesh::Point w) {
	v.normalize();
	w.normalize();
	if (v == w)
		return std::numeric_limits<float>::infinity();

	return(v.dot(w) / v.cross(w).norm());
};

#pragma region MyMesh

MyMesh::MyMesh()
{
	request_vertex_normals();
	request_vertex_status();
	request_vertex_texcoords2D();

	request_edge_status();
	request_halfedge_status();

	request_face_status();

	this->add_property(prop_G, "prop_G"); 
	this->add_property(prop_W, "prop_W");
	this->add_property(prop_W_C, "prop_W_C");
}

MyMesh::~MyMesh()
{

}

void MyMesh::Reset()
{
	deformed_vertices.clear();
	for (MyMesh::VertexIter v_it = vertices_begin(); v_it != vertices_end(); ++v_it)
	{
		deformed_vertices.push_back(point(v_it));
	}
	for (int i = 0; i < controlPoints.size(); i++) {
		controlPoints[i].c = controlPoints[i].o;
	}
}

void MyMesh::Initialization()
{
	std::cout << "Vert : " << n_vertices() << std::endl;
	std::cout << "Face : " << n_faces() << std::endl;

	deformed_vertices.clear();
	for (MyMesh::VertexIter v_it = vertices_begin(); v_it != vertices_end(); ++v_it)
	{
		deformed_vertices.push_back(point(v_it));
	}

	Registration();
	InitCompilation();
}

void MyMesh::Registration()
{
	// precompute L1, L2 and G
	preComputeG();
	preComputeL1();
	preComputeL2();
}

void MyMesh::preComputeG()
{
	Eigen::MatrixXd boundary_local(4,6);
	boundary_local <<
		-1.0, 0.0, 1.0, 0.0, 0.0, 0.0,
		0.0, -1.0, 0.0, 1.0, 0.0, 0.0,
		-1.0, 0.0, 0.0, 0.0, 1.0, 0.0,
		0.0, -1.0, 0.0, 0.0, 0.0, 1.0;

	Eigen::MatrixXd inside_local(6,8);
	inside_local <<
		-1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
		0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
		-1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
		0.0, -1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
		-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
		0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;

	// pre G
	for (auto e_it = this->edges_begin(); e_it != this->edges_end(); ++e_it) {
		int rows = (is_boundary(e_it) ? 4 : 6);
		int row = 0;

		Eigen::MatrixXd G(rows, 2);
		double Weight = 0;

		HalfedgeHandle heh = halfedge_handle(e_it, 0);
		MyMesh::Point pFrom = point(from_vertex_handle(heh));
		/*G(row, 0) = pFrom[0]; G(row, 1) = pFrom[2]; row += 1;
		G(row, 0) = pFrom[2]; G(row, 1) = -pFrom[0]; row += 1;*/

		MyMesh::Point pTo = point(to_vertex_handle(heh));
		MyMesh::Point vTo = pTo - pFrom;
		G(row, 0) = vTo[0]; G(row, 1) = vTo[2]; row += 1;
		G(row, 0) = vTo[2]; G(row, 1) = -vTo[0]; row += 1;

		// boundary check
		VertexHandle vh0 = opposite_vh(heh);
		if (vh0 != MyMesh::InvalidVertexHandle) {
			MyMesh::Point p0 = point(vh0);
			MyMesh::Point v0 = p0 - pFrom;
			G(row, 0) = v0[0]; G(row, 1) = v0[2]; row += 1;
			G(row, 0) = v0[2]; G(row, 1) = -v0[0]; row += 1;
			Weight += abs(cot(pTo - p0, pFrom - p0));
		}

		VertexHandle vh1 = opposite_he_opposite_vh(heh);
		if (vh1 != MyMesh::InvalidVertexHandle) {
			MyMesh::Point p1 = point(vh1);
			MyMesh::Point v1 = p1 - pFrom;
			G(row, 0) = v1[0]; G(row, 1) = v1[2]; row += 1;
			G(row, 0) = v1[2]; G(row, 1) = -v1[0];

			Weight += abs(cot(pTo - p1, pFrom - p1));
		}

		if (!is_boundary(e_it)) {
			Weight *= 0.5;
			this->property(prop_G, e_it) = ((G.transpose() * G).inverse() * G.transpose()) * inside_local;
		}
		else {
			this->property(prop_G, e_it) = ((G.transpose() * G).inverse() * G.transpose()) * boundary_local;
		}

		this->property(prop_W, e_it) = Weight;
		this->property(prop_W_C, e_it) = 0;
	}
}

void MyMesh::preComputeL1()
{
	const int N_E(n_edges());
	const int N_V(n_vertices());

	L1 = Eigen::SparseMatrix<double>(N_E * 2, N_V * 2);
	std::vector<Eigen::Triplet<double>> triplet_list_L1;

	for (auto e_it = this->edges_begin(); e_it != this->edges_end(); ++e_it) {

		int cols = (is_boundary(e_it) ? 6 : 8);
		int row = e_it->idx() * 2;

		HalfedgeHandle heh = halfedge_handle(e_it, 0);

		Eigen::MatrixXd& G = this->property(prop_G, e_it);

		VertexHandle vh_from = from_vertex_handle(heh);
		VertexHandle vh_to = to_vertex_handle(heh);

		// edge vector
		MyMesh::Point pFrom = point(vh_from);
		MyMesh::Point e = point(vh_to) - pFrom;
		Eigen::Matrix2d e_mat;
		e_mat << e[0], e[2],
			e[2], -e[0];

		Eigen::MatrixXd h = Eigen::MatrixXd::Zero(2, cols);
		h(0, 0) = -1;
		h(0, 2) = 1;
		h(1, 1) = -1;
		h(1, 3) = 1;

		h = h - e_mat * G;

		double Weight = std::max(this->property(prop_W, e_it), this->property(prop_W_C, e_it));
		h *= Weight;

		int col = vh_from.idx() * 2;
		int hcol = 0;
		triplet_list_L1.push_back(Eigen::Triplet<double>(row, col, h(0, hcol)));
		triplet_list_L1.push_back(Eigen::Triplet<double>(row, col + 1, h(0, hcol + 1)));
		triplet_list_L1.push_back(Eigen::Triplet<double>(row + 1, col, h(1, hcol)));
		triplet_list_L1.push_back(Eigen::Triplet<double>(row + 1, col + 1, h(1, hcol + 1)));

		col = vh_to.idx() * 2;
		hcol += 2;
		triplet_list_L1.push_back(Eigen::Triplet<double>(row, col, h(0, hcol)));
		triplet_list_L1.push_back(Eigen::Triplet<double>(row, col + 1, h(0, hcol + 1)));
		triplet_list_L1.push_back(Eigen::Triplet<double>(row + 1, col, h(1, hcol)));
		triplet_list_L1.push_back(Eigen::Triplet<double>(row + 1, col + 1, h(1, hcol + 1)));

		// boundary check
		VertexHandle vh0 = opposite_vh(heh);
		if (vh0 != MyMesh::InvalidVertexHandle) {
			col = vh0.idx() * 2;
			hcol += 2;
			triplet_list_L1.push_back(Eigen::Triplet<double>(row, col, h(0, hcol)));
			triplet_list_L1.push_back(Eigen::Triplet<double>(row, col + 1, h(0, hcol + 1)));
			triplet_list_L1.push_back(Eigen::Triplet<double>(row + 1, col, h(1, hcol)));
			triplet_list_L1.push_back(Eigen::Triplet<double>(row + 1, col + 1, h(1, hcol + 1)));
		}

		VertexHandle vh1 = opposite_he_opposite_vh(heh);
		if (vh1 != MyMesh::InvalidVertexHandle) {
			col = vh1.idx() * 2;
			hcol += 2;
			triplet_list_L1.push_back(Eigen::Triplet<double>(row, col, h(0, hcol)));
			triplet_list_L1.push_back(Eigen::Triplet<double>(row, col + 1, h(0, hcol + 1)));
			triplet_list_L1.push_back(Eigen::Triplet<double>(row + 1, col, h(1, hcol)));
			triplet_list_L1.push_back(Eigen::Triplet<double>(row + 1, col + 1, h(1, hcol + 1)));
		}
	}

	L1.setFromTriplets(triplet_list_L1.begin(), triplet_list_L1.end());
	LL1 = L1.transpose() * L1;
}

void MyMesh::preComputeL2()
{
	const int N_E(n_edges());
	const int N_V(n_vertices());

	L2 = Eigen::SparseMatrix<double>(N_E, N_V);
	std::vector<Eigen::Triplet<double>> triplet_list_L2;

	for (auto e_it = this->edges_begin(); e_it != this->edges_end(); ++e_it) {

		int row = e_it->idx();

		HalfedgeHandle heh = halfedge_handle(e_it, 0);

		VertexHandle vh_from = from_vertex_handle(heh);
		VertexHandle vh_to = to_vertex_handle(heh);

		double Weight = std::max(this->property(prop_W, e_it), this->property(prop_W_C, e_it));

		triplet_list_L2.push_back(Eigen::Triplet<double>(row, vh_from.idx(), -1 * Weight));
		triplet_list_L2.push_back(Eigen::Triplet<double>(row, vh_to.idx(), 1 * Weight));
	}

	L2.setFromTriplets(triplet_list_L2.begin(), triplet_list_L2.end());
	LL2 = L2.transpose() * L2;
}

void MyMesh::select(unsigned int face_ID, MyMesh::Point p)
{
	FaceHandle fh = this->face_handle(face_ID);

	FaceVertexIter fv_it = fv_iter(fh);
	MyMesh::Point& dp0 = deformed_vertices[fv_it->idx()];
	MyMesh::Point p0 = point(fv_it); ++fv_it;

	MyMesh::Point& dp1 = deformed_vertices[fv_it->idx()];
	MyMesh::Point p1 = point(fv_it); ++fv_it;

	MyMesh::Point& dp2 = deformed_vertices[fv_it->idx()];
	MyMesh::Point p2 = point(fv_it);

	MyMesh::Point vp0 = dp0 - p;
	MyMesh::Point vp1 = dp1 - p;
	MyMesh::Point vp2 = dp2 - p;

	double a0 = vp1.cross(vp2).length();
	double a1 = vp0.cross(vp2).length();
	double a2 = vp0.cross(vp1).length();

	double i_area = 1.0 / (a0 + a1 + a2);

	ControlPoint cp;
	cp.fh = fh;
	cp.w[0] = i_area * a0;
	cp.w[1] = i_area * a1;
	cp.w[2] = i_area * a2;
	cp.o = cp.w[0] * p0 + cp.w[1] * p1 + cp.w[2] * p2;
	cp.c = cp.w[0] * dp0 + cp.w[1] * dp1 + cp.w[2] * dp2;

	//std::cout << cp.o << std::endl;
	//cp.c = MyMesh::Point(0,0,0);

	AddControlPoint(cp);
}

void MyMesh::InitCompilation()
{
	controlPoints.clear();
	C1_triplets.clear();
	C2_triplets.clear();
}

void MyMesh::AddControlPoint(ControlPoint cp)
{
	FaceVertexIter fv_it = fv_iter(cp.fh);
	int p0_idx = fv_it->idx(); ++fv_it;
	int p1_idx = fv_it->idx(); ++fv_it;
	int p2_idx = fv_it->idx();

	int row = controlPoints.size();

	int c1x_idx = row * 2;
	int c1y_idx = row * 2 + 1;
	C1_triplets.push_back(Eigen::Triplet<double>(c1x_idx, p0_idx * 2, cp.w[0] * W));
	C1_triplets.push_back(Eigen::Triplet<double>(c1y_idx, p0_idx * 2 + 1, cp.w[0] * W));

	C1_triplets.push_back(Eigen::Triplet<double>(c1x_idx, p1_idx * 2, cp.w[1] * W));
	C1_triplets.push_back(Eigen::Triplet<double>(c1y_idx, p1_idx * 2 + 1, cp.w[1] * W));

	C1_triplets.push_back(Eigen::Triplet<double>(c1x_idx, p2_idx * 2, cp.w[2] * W));
	C1_triplets.push_back(Eigen::Triplet<double>(c1y_idx, p2_idx * 2 + 1, cp.w[2] * W));

	int c2_idx = row;
	C2_triplets.push_back(Eigen::Triplet<double>(c2_idx, p0_idx, cp.w[0] * W));
	C2_triplets.push_back(Eigen::Triplet<double>(c2_idx, p1_idx, cp.w[1] * W));
	C2_triplets.push_back(Eigen::Triplet<double>(c2_idx, p2_idx, cp.w[2] * W));

	controlPoints.push_back(cp);
	Compilation();
}

void MyMesh::AddControlPoints(std::vector<ControlPoint>& cps)
{
	holdCompilation = true;
	for (int i = 0; i < cps.size(); i++) {
		AddControlPoint(cps[i]);
	}
	holdCompilation = false;
	Compilation();
}

void MyMesh::RemoveControlPoint(unsigned int idx)
{
	int n_controlPoints = controlPoints.size();
	int s_id = (n_controlPoints - 1);

	ControlPoint& cp = controlPoints[idx];

	for (int i = 0; i < 3; i++) {
		int c1_idx = idx * 6 + i * 2;
		int c1_sidx = s_id * 6 + i * 2;
		C1_triplets[c1_idx] = Eigen::Triplet<double>(C1_triplets[c1_idx].row(), C1_triplets[c1_sidx].col(), C1_triplets[c1_sidx].value());
		C1_triplets[c1_idx+1]= Eigen::Triplet<double>(C1_triplets[c1_idx+1].row(), C1_triplets[c1_sidx+1].col(), C1_triplets[c1_sidx].value());

		int c2_idx = idx * 3 + i;
		int c2_sidx = s_id * 3 + i;
		C2_triplets[c2_idx] = Eigen::Triplet<double>(C2_triplets[c2_idx].row(), C2_triplets[c2_sidx].col(), C2_triplets[c2_sidx].value());
	}

	C1_triplets.erase(C1_triplets.end() - 6, C1_triplets.end());
	C2_triplets.erase(C2_triplets.end() - 3, C2_triplets.end());

	controlPoints[idx] = controlPoints[s_id];
	controlPoints.erase(controlPoints.end() - 1);

	if (!controlPoints.empty()) {
		Compilation();
	}
}

void MyMesh::Compilation()
{
	if (holdCompilation)
		return;

	const int N_V(n_vertices());
	const int N_E(n_edges());
	const int N_C(controlPoints.size());

	C1.resize(N_C * 2, N_V * 2); // solve x,y together
	C2.resize(N_C, N_V); // solve x, y respectively

	C1.setFromTriplets(C1_triplets.begin(), C1_triplets.end());
	C2.setFromTriplets(C2_triplets.begin(), C2_triplets.end());

	CC1 = C1.transpose() * C1;
	CC2 = C2.transpose() * C2;

	A1.resize(N_E * 2 + N_C * 2, N_V * 2);
	A1.topRows(N_E * 2) = L1;
	A1.bottomRows(N_C * 2) = C1;

	AA1 = LL1 + CC1;
	solver1.analyzePattern(AA1);
	solver1.factorize(AA1);

	b1.resize(N_E * 2 + N_C * 2, 1);
	b1 = Eigen::MatrixXd::Zero(N_E * 2 + N_C * 2, 1);

	A2.resize(N_E + N_C, N_V);
	A2.topRows(N_E) = L2;
	A2.bottomRows(N_C) = C2;

	AA2 = LL2 + CC2;
	solver2.analyzePattern(AA2);
	solver2.factorize(AA2);

	b2x.resize(N_E + N_C, 1);
	b2x = Eigen::MatrixXd::Zero(N_E + N_C, 1);
	b2y.resize(N_E + N_C, 1);
	b2y = Eigen::MatrixXd::Zero(N_E + N_C, 1);

}

void MyMesh::SetEdgeWeights(std::set<unsigned int>& faces)
{
	const float W = 90.3f;

	for (auto e_it = this->edges_begin(); e_it != this->edges_end(); ++e_it) {
		this->property(prop_W_C, e_it) = 0;
	}
	for (auto f_it = faces.begin(); f_it != faces.end(); ++f_it) {
		FaceHandle f_h = face_handle(*f_it);

		for (FaceEdgeIter fe_it = fe_iter(f_h); fe_it.is_valid(); ++fe_it) {
			this->property(prop_W_C, fe_it) = W;
		}
	}

	preComputeL1();
	preComputeL2();
	Compilation();
}

void MyMesh::Compute(unsigned int id)
{
	//std::cout << id << std::endl;

	if (id == -1) {
		std::cout << "YOYYO" << std::endl;
	}

	int N_V(n_vertices());
	int N_C = controlPoints.size();

	if (N_V == 0 || N_C == 0)
		return;

	if (N_C == 1) {
		Point offset = (controlPoints[0].c - controlPoints[0].o);
		for (MyMesh::VertexIter v_it = vertices_begin(); v_it != vertices_end(); ++v_it)
		{
			deformed_vertices[v_it->idx()] = point(v_it) + offset;
		}
		return;
	}

	Step1();
	Step2();


	for (MyMesh::VertexIter v_it = vertices_begin(); v_it != vertices_end(); ++v_it)
	{
		float depth = 0;

		MyMesh::Point d0(V2x(v_it->idx()), 0, V2y(v_it->idx()));
		MyMesh::Point p0 = point(v_it);

		for (VertexVertexIter vv_it = vv_iter(v_it); vv_it.is_valid(); ++vv_it) {

			MyMesh::Point d1(V2x(vv_it->idx()), 0, V2y(vv_it->idx()));
			depth += (point(vv_it) - p0).normalized().dot((d1 - d0).normalized());
		}

		d0[1] = depth + p0.length() * 0.1f;
		deformed_vertices[v_it->idx()] = d0;
	}
}

void MyMesh::Step1()
{
	const int N_V(n_vertices());
	const int N_E(n_edges());
	const int N_C(controlPoints.size());

	for (int i = 0; i < controlPoints.size(); i++) {
		b1(N_E * 2 + i * 2, 0) = controlPoints[i].c[0] * W;
		b1(N_E * 2 + i * 2 + 1, 0) = controlPoints[i].c[2] * W;
	}

	//Eigen::SparseLU< Eigen::SparseMatrix<double>> solver(AA1);
	//Eigen::SimplicialCholesky<Eigen::SparseMatrix<double>> solver(AA1);

	V1 = solver1.solve(A1.transpose() * b1);
}

void MyMesh::Step2()
{
	const int N_V(n_vertices());
	const int N_E(n_edges());
	const int N_C(controlPoints.size());

	
	/*V2x = Eigen::VectorXd(N_V);
	V2y = Eigen::VectorXd(N_V);

	for (int i = 0; i < N_V; i++) {
		V2x(i) = V1(i * 2);
		V2y(i) = V1(i * 2 + 1);
	}
	return;*/
	

	for (auto e_it = this->edges_begin(); e_it != this->edges_end(); ++e_it) {

		int cols = (is_boundary(e_it) ? 6 : 8);
		int row = e_it->idx();

		HalfedgeHandle heh = halfedge_handle(e_it, 0);

		Eigen::MatrixXd& G = this->property(prop_G, e_it);
		double Weight = std::max(this->property(prop_W, e_it), this->property(prop_W_C, e_it));

		VertexHandle vh_from = from_vertex_handle(heh);
		VertexHandle vh_to = to_vertex_handle(heh);

		double pFromX = V1(vh_from.idx() * 2);
		double pFromY = V1(vh_from.idx() * 2 + 1);

		double pToX = V1(vh_to.idx() * 2);
		double pToY = V1(vh_to.idx() * 2 + 1);

		double c = 0;
		double s = 0;

		c += G(0, 0) * pFromX + G(0, 1) * pFromY; c += G(0, 2) * pToX + G(0, 3) * pToY;
		s += G(1, 0) * pFromX + G(1, 1) * pFromY; s += G(1, 2) * pToX + G(1, 3) * pToY;

		int v_row = 2;
		// boundary check
		VertexHandle vh0 = opposite_vh(heh);
		if (vh0 != MyMesh::InvalidVertexHandle) {
			double p0X = V1(vh0.idx() * 2);
			double p0Y = V1(vh0.idx() * 2 + 1);
			c += G(0, v_row * 2) * p0X + G(0, v_row * 2 + 1) * p0Y;
			s += G(1, v_row * 2) * p0X + G(1, v_row * 2 + 1) * p0Y;
			v_row += 1;
		}
		VertexHandle vh1 = opposite_he_opposite_vh(heh);
		if (vh1 != MyMesh::InvalidVertexHandle) {
			double p1X = V1(vh1.idx() * 2);
			double p1Y = V1(vh1.idx() * 2 + 1);
			c += G(0, v_row * 2) * p1X + G(0, v_row * 2 + 1) * p1Y;
			s += G(1, v_row * 2) * p1X + G(1, v_row * 2 + 1) * p1Y;
		}

		MyMesh::Point e = point(vh_to) - point(vh_from);
		double det = (c * c + s * s);
		if (det == 0) {
			b2x(row, 0) = e[0] * Weight;
			b2y(row, 0) = e[2] * Weight;
		}
		else {
			double norm = 1.0 / sqrt(det);
			b2x(row, 0) = (e[0] * c + e[2] * s) * norm * Weight;
			b2y(row, 0) = (e[2] * c - e[0] * s) * norm * Weight;
		}
	}

	for (int i = 0; i < controlPoints.size(); i++) {
		b2x(N_E + i, 0) = controlPoints[i].c[0] * W;
		b2y(N_E + i, 0) = controlPoints[i].c[2] * W;
	}

	V2x = solver2.solve(A2.transpose() * b2x);
	V2y = solver2.solve(A2.transpose() * b2y);
}

#pragma endregion

#pragma region GLMesh

GLMesh::GLMesh()
{
	current_key = MyMesh::Point(0, 0, 0);
	/*
	sc = new serverController();
	sc->Stop();
	std::function<void(char*, int)> callback = [this](char* buffer, int length) { this->socketCallback(buffer, length); };
	sc->Run(callback);*/
}

GLMesh::~GLMesh()
{
	/*sc->Stop();*/
}

bool GLMesh::Init(std::string fileName)
{
	std::string filetype = fileName.substr(fileName.find_last_of(".") + 1);
	bool success = false;

	mesh.clear();
	keyPoints.clear();
	keyData.clear();
	constrainedTriIDs.clear();

	texture = nullptr;

	if (filetype == "bmp" || filetype == "jpg" || filetype == "png") {
		success = Load2DImage(fileName);
	}
	else if (filetype == "obj") {
		success = LoadMesh(fileName);
	}
	else if (filetype == "txt") {
		success = Load2DModel(fileName);
	}

	if (success)
	{
		glGenVertexArrays(1, &this->vao.vao);
		glBindVertexArray(this->vao.vao);

		glGenBuffers(3, this->vao.vbo);

		glBindBuffer(GL_ARRAY_BUFFER, this->vao.vbo[0]);
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);
		glEnableVertexAttribArray(0);

		glBindBuffer(GL_ARRAY_BUFFER, this->vao.vbo[1]);
		glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, 0);
		glEnableVertexAttribArray(1);

		glGenBuffers(1, &this->vao.ebo);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, this->vao.ebo);

		glBindBuffer(GL_ARRAY_BUFFER, 0);
		glBindVertexArray(0);

		LoadToShader();
		LoadTexCoordToShader();

		std::cout << "SUCCESS" << std::endl;
		return true;
	}

	std::cout << "FAILED" << std::endl;
	return false;
}

void GLMesh::renderMesh()
{
	if (this->vao.element_amount > 0)
	{
		glEnable(GL_TEXTURE_2D);

		if (texture != nullptr)
			texture->bind(0);

		glBindVertexArray(this->vao.vao);
		glDrawElements(GL_TRIANGLES, this->vao.element_amount, GL_UNSIGNED_INT, 0);
		glBindVertexArray(0);

		if (texture != nullptr)
			texture->unbind(0);

		glDisable(GL_TEXTURE_2D);
	}
}
void GLMesh::renderSelectedMesh()
{
	if (!constrainedTriIDs.empty())
	{
		std::vector<unsigned int*> offsets;
		for (auto tri_it = constrainedTriIDs.begin(); tri_it != constrainedTriIDs.end(); ++tri_it)
		{
			unsigned int tri_id = *tri_it;
			offsets.push_back((GLuint*)(tri_id * 3 * sizeof(GLuint)));
		}
		std::vector<int> count(constrainedTriIDs.size(), 3);
		glBindVertexArray(this->vao.vao);
		glMultiDrawElements(GL_TRIANGLES, &count[0], GL_UNSIGNED_INT, (const GLvoid**)&offsets[0], constrainedTriIDs.size());
		glBindVertexArray(0);
	}
}
void GLMesh::renderControlPoints()
{
	//glEnable(GL_PROGRAM_POINT_SIZE);
	glPointSize(10);
	glEnable(GL_POINT_SMOOTH);
	glBegin(GL_POINTS);
	size_t n_controlPoints = this->mesh.controlPoints.size();
	for (size_t i = 0; i < n_controlPoints; i++) {
		if (i == select_id) {
			glColor3d(1, 1, 0);
		}
		else {
			glColor3d(0.2, 0.2, 0.9);
		}
		MyMesh::ControlPoint& cp = this->mesh.controlPoints[i];
		glVertex3f(cp.c[0], 0, cp.c[2]);
	}
	glEnd();
	glDisable(GL_POINT_SMOOTH);
	//glDisable(GL_PROGRAM_POINT_SIZE);
}

void GLMesh::renderKeyPoints()
{
	glPointSize(15);
	glEnable(GL_POINT_SMOOTH);
	glBegin(GL_POINTS);

	glColor3d(0.05, 0.05, 0.9);
	for (size_t i = 0; i < keyPoints.size(); i++) {
		if (i != select_k_id) {
			glVertex3f(keyPoints[i][0], keyPoints[i][1], keyPoints[i][2]);
		}
	}

	// draw select point
	if (select_k_id != -1) {
		glColor3d(0.9, 0.9, 0);
		glVertex3f(keyPoints[select_k_id][0], keyPoints[select_k_id][1], keyPoints[select_k_id][2]);
	}
	glEnd();

	
	glPointSize(11);
	glBegin(GL_POINTS);
	// draw current key
	glColor3d(0.9, 0.05, 0.05);
	glVertex3f(current_key[0], current_key[1], current_key[2]);

	glEnd();
	glDisable(GL_POINT_SMOOTH);
}

void GLMesh::select(unsigned int tri_ID, MyMesh::Point p)
{
	this->mesh.select(tri_ID, p);
}

void GLMesh::selectTri(unsigned int tri_ID, bool state)
{
	if (state) {
		constrainedTriIDs.insert(tri_ID);
	}
	else {
		constrainedTriIDs.erase(tri_ID);
	}

	edge_weight_modified = true;
}

void GLMesh::applyTriangleWeights()
{
	if(edge_weight_modified)
		mesh.SetEdgeWeights(constrainedTriIDs);

	edge_weight_modified = false;
}

// control points
void GLMesh::selectControlPoint(MyMesh::Point p)
{
	select_id = -1;
	float min_d = 20;
	size_t n_controlPoints = this->mesh.controlPoints.size();
	for (size_t i = 0; i < n_controlPoints; i++) {
		MyMesh::ControlPoint& cp = this->mesh.controlPoints[i];

		float d = (cp.c - p).length();
		if (d < min_d) {
			min_d = d;
			select_id = i;
		}
	}
}

void GLMesh::dragControlPoint(MyMesh::Point p)
{
	if (select_id == -1)
		return;

	this->mesh.controlPoints[select_id].c = p;
	if (select_k_id != -1) {
		keyData[select_k_id][select_id] = p;
	}
	this->mesh.Compute(select_id);
	UpdateShader();
}

void GLMesh::remove_selected()
{
	if (validID(select_id)) {
		mesh.RemoveControlPoint(select_id);
	}
	select_id = -1;
}


// keypoints
void GLMesh::addKeyPoint(MyMesh::Point p)
{
	std::vector<MyMesh::Point> cps;
	
	size_t n_controlPoints = this->mesh.controlPoints.size();
	for (size_t i = 0; i < n_controlPoints; i++) {
		MyMesh::ControlPoint& cp = this->mesh.controlPoints[i];
		cps.push_back(cp.c);
	}

	p[0] = int(p[0]);
	p[1] = int(p[1]);
	p[2] = int(p[2]);

	keyPoints.push_back(p);
	keyData.push_back(cps);

}
void GLMesh::selectKeyPoint(MyMesh::Point p)
{
	select_k_id = -1;
	float min_d = 0.4;

	float d = (current_key - p).length();
	if (d < min_d) {
		min_d = d;
		select_k_id = -1;
		return;
	}

	for (size_t i = 0; i < keyPoints.size(); i++) {
		MyMesh::Point& kp = keyPoints[i];
		d = (kp - p).length();
		if (d < min_d) {
			min_d = d;
			select_k_id = i;
		}
	}

	if (select_k_id != -1) {
		size_t n_controlPoints = this->mesh.controlPoints.size();

		for (size_t j = 0; j < n_controlPoints; j++) {
			MyMesh::ControlPoint& cp = this->mesh.controlPoints[j];
			cp.c = keyData[select_k_id][j];
		}
		mesh.Compute(0);
		UpdateShader();
	}
	
}
void GLMesh::dragKeyPoint(MyMesh::Point p)
{
	if (select_k_id == -1) {
		current_key = p;
		Interpolate();
	}
	else {
		keyPoints[select_k_id][0] = int(p[0]);
		keyPoints[select_k_id][1] = int(p[1]);
		keyPoints[select_k_id][2] = int(p[2]);
	}
}
void GLMesh::removeKeyPoint()
{
	if (select_k_id != -1) {
		keyPoints.erase(keyPoints.begin() + select_k_id);
		keyData.erase(keyData.begin() + select_k_id);
	}
	select_k_id = -1;
}

void GLMesh::Interpolate()
{
	size_t n_controlPoints = this->mesh.controlPoints.size();
	size_t n_keyPoints = keyPoints.size();

	if (n_controlPoints == 0 || n_keyPoints == 0) {
		return;
	}

	float weight_sum = 0;
	float epsilon = 0.0001f;
	for (size_t j = 0; j < n_controlPoints; j++) {
		MyMesh::ControlPoint& cp = this->mesh.controlPoints[j];
		cp.c = MyMesh::Point(0, 0, 0);
	}

	for (size_t i = 0; i < n_keyPoints; i++) {
		MyMesh::Point& kp = keyPoints[i];
		float weight = 1.0 / ((kp - current_key).sqrnorm() + epsilon);
		weight_sum += weight;
		for (size_t j = 0; j < n_controlPoints; j++) {
			MyMesh::ControlPoint& cp = this->mesh.controlPoints[j];
			cp.c += (keyData[i][j] * weight);
		}
	}

	for (size_t j = 0; j < n_controlPoints; j++) {
		MyMesh::ControlPoint& cp = this->mesh.controlPoints[j];
		cp.c /= weight_sum;
	}

	mesh.Compute(0);
	UpdateShader();
}

#pragma region Connection
void GLMesh::socketCallback(char* buffer, int length)
{
	if (is_decoding)
		return;

	is_decoding = true;

	float scale_up = 1.4f;

	float x, y;
	int idx = 0;
	int N_C = mesh.controlPoints.size();
	std::reverse(buffer, buffer + length);
	for (int i = 0; i < length && idx < N_C; i += 8, idx += 1) {
		char* y_pos = buffer + i;
		char* x_pos = buffer + i + 4;

		memcpy(&x, x_pos, sizeof(float));
		memcpy(&y, y_pos, sizeof(float));

		// x,y is [0 , 1]
		mesh.controlPoints[N_C-1-idx].c[0] = (x * 2 - 1) * SIZE * scale_up;
		mesh.controlPoints[N_C-1-idx].c[2] = -(y * 2 - 1) * SIZE * scale_up;

		//std::cout << x << ", " << y << ", "<< mesh.controlPoints[N_C - 1 - idx].c[0] << ", " << mesh.controlPoints[N_C - 1 - idx].c[2] << std::endl;
	}

	is_changed[0] = true;
	//std::cout << "end" << std::endl;
	is_decoding = false;
}

void GLMesh::checkUpdate()
{
	if (is_changed[0]) {
		is_changed[0] = false;
		mesh.Compute(0);
		UpdateShader();
	}
}
#pragma endregion


#pragma region Utilities
bool GLMesh::validID(unsigned int faceID)
{
	return (faceID < mesh.n_faces());
}

void GLMesh::LoadToShader()
{
	std::vector<MyMesh::Point> vertices;
	std::vector<MyMesh::Normal> normals;
	for (MyMesh::VertexIter v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); ++v_it)
	{
		vertices.push_back(mesh.point(*v_it));
		normals.push_back(mesh.normal(*v_it));
	}
	std::vector<unsigned int> indices;
	for (MyMesh::FaceIter f_it = mesh.faces_begin(); f_it != mesh.faces_end(); ++f_it)
		for (MyMesh::FaceVertexIter fv_it = mesh.fv_iter(*f_it); fv_it.is_valid(); ++fv_it)
			indices.push_back(fv_it->idx());

	LoadToShader(vertices, normals, indices);
}
void GLMesh::LoadToShader(
	std::vector<MyMesh::Point>& vertices,
	std::vector<MyMesh::Normal>& normals,
	std::vector<unsigned int>& indices)
{
	if (!vertices.empty()) {
		glBindBuffer(GL_ARRAY_BUFFER, this->vao.vbo[0]);
		glBufferData(GL_ARRAY_BUFFER, sizeof(MyMesh::Point) * vertices.size(), &vertices[0], GL_STATIC_DRAW);
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);
		glEnableVertexAttribArray(0);
	}
	if (!normals.empty()) {
		glBindBuffer(GL_ARRAY_BUFFER, this->vao.vbo[1]);
		glBufferData(GL_ARRAY_BUFFER, sizeof(MyMesh::Normal) * normals.size(), &normals[0], GL_STATIC_DRAW);
		glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, 0);
		glEnableVertexAttribArray(1);
	}
	if (!indices.empty()) {
		this->vao.element_amount = indices.size();
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, this->vao.ebo);
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned int) * indices.size(), &indices[0], GL_STATIC_DRAW);
	}
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindVertexArray(0);
}

void GLMesh::UpdateShader() {
	std::vector<MyMesh::Normal> normals;
	std::vector<unsigned int> indices;
	LoadToShader(mesh.deformed_vertices, normals, indices);
}

void GLMesh::resetMesh()
{
	current_key = MyMesh::Point(0, 0, 0);
	this->mesh.Reset();
	std::vector<MyMesh::Normal> normals;
	std::vector<unsigned int> indices;
	LoadToShader(mesh.deformed_vertices, normals, indices);
}

#pragma endregion

#pragma region IO
typedef CGAL::Delaunay_mesher_2<CGAL_CDT, CGAL_Criteria> Mesher;
bool GLMesh::Load2DImage(std::string fileName)
{
	cv::Mat rgb_img = cv::imread(fileName);

	cv::Mat	img;
	cv::cvtColor(rgb_img, img, CV_RGB2GRAY);
	cv::threshold(img, img, 254, 255, cv::ThresholdTypes::THRESH_BINARY);
	//cv::Canny(img, edges, 100, 210);
	//floodFill(edges, cv::Point2i(edges.cols / 2, edges.rows / 2), cv::Scalar(255, 255, 255));

	std::vector<std::vector<cv::Point>> contour;
	cv::findContours(img, contour, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);

	if (contour.size() == 0)
		return false;

	int contour_id = 0;
	int max = 0;
	for (int i = 0; i < contour.size(); i++) {
		if (contour[i].size() > max) {
			max = contour[i].size();
			contour_id = i;
		}
	}

	float epsilon = 0.002 * cv::arcLength(contour[contour_id], true);

	std::vector<cv::Point> approx;
	cv::approxPolyDP(contour[contour_id], approx, epsilon, true);

	// find bounding box
	int max_x = approx[0].x;
	int max_y = approx[0].y;
	int min_x = approx[0].x;
	int min_y = approx[0].y;
	for (int i = 0; i < approx.size(); i++)
	{
		max_x = std::max(approx[i].x, max_x);
		max_y = std::max(approx[i].y, max_y);
		min_x = std::min(approx[i].x, min_x);
		min_y = std::min(approx[i].y, min_y);
	}

	float norm_size = 1.0f;
	float norm_scale = norm_size / std::max(abs(max_x - min_x), abs(max_y - min_y));
	float x_offset = (max_x + min_x) * norm_scale * 0.5f;
	float y_offset = (max_y + min_y) * norm_scale * 0.5f;

	// create constrainted delaunay triangulation handler
	CGAL_CDT cdt;

	// insertion
	std::vector<CGAL_Vertex_handle> vertices;
	for (int i = 0; i < approx.size(); i++)
	{
		vertices.push_back(
			cdt.insert(CGAL_Point(approx[i].x * norm_scale - x_offset, (1 - approx[i].y * norm_scale) - y_offset))
		);
	}

	for (std::size_t i = 1; i < approx.size(); ++i)
	{
		cdt.insert_constraint(vertices[i-1], vertices[i]);
	}
	cdt.insert_constraint(vertices[0], vertices[approx.size() -1]);

	std::cout << "Number of vertices: " << cdt.number_of_vertices() << std::endl;
	Mesher mesher(cdt);
	std::cout << "Meshing with criterias..." << std::endl;
	mesher.set_criteria(CGAL_Criteria(0.125, 0.08));
	mesher.refine_mesh();
	std::cout << " done." << std::endl;

	if (cdt.number_of_vertices() == 0)
		return false;

	float scale = SIZE;
	std::map<CGAL_Vertex_handle, MyMesh::VertexHandle> v_handles;
	for (auto v_it = cdt.finite_vertices_begin(); v_it != cdt.finite_vertices_end(); ++v_it)
	{
		CGAL_Vertex_handle h = v_it->handle();
		auto& p = v_it->point();

		OpenMesh::Vec3f v(p.x() * scale, 0, p.y() * scale);
		MyMesh::VertexHandle v_h = mesh.add_vertex(v);
		mesh.set_texcoord2D(v_h, MyMesh::TexCoord2D(((p.x() + x_offset) / norm_scale) / img.cols, ((1 - p.y() - y_offset) / norm_scale) / img.rows));

		v_handles[h] = v_h;
	}

	std::vector<MyMesh::VertexHandle> face_vhandles;
	for (auto f_it = cdt.finite_faces_begin(); f_it != cdt.finite_faces_end(); ++f_it)
	{
		if (f_it->is_in_domain()) {

			CGAL_Vertex_handle h0 = f_it->vertex(0)->handle();
			CGAL_Vertex_handle h1 = f_it->vertex(1)->handle();
			CGAL_Vertex_handle h2 = f_it->vertex(2)->handle();

			face_vhandles.clear();
			face_vhandles.push_back(v_handles[h0]);
			face_vhandles.push_back(v_handles[h1]);
			face_vhandles.push_back(v_handles[h2]);

			mesh.add_face(face_vhandles);
		}
	}

	mesh.Initialization();

	texture = new Texture2D(fileName.c_str());

	return true;
}
bool GLMesh::Load2DModel(std::string fileName)
{
	std::ifstream ifs(fileName);
	if (!ifs.is_open()) {
		std::cout << "Cannot open file \"" << fileName << "\" !!" << std::endl;
		return false;
	}

	std::size_t nPts, nEdges;
	ifs >> nPts >> nEdges;

	std::vector<MyMesh::Point> m_points;

	for (std::size_t i = 0; i < nPts; ++i)
	{
		float x1, y1;
		ifs >> x1 >> y1;
		m_points.push_back(MyMesh::Point(x1, y1, 0));
	}
	// find bounding box
	float max_x = m_points[0][0];
	float max_y = m_points[0][1];
	float min_x = m_points[0][0];
	float min_y = m_points[0][1];
	for (int i = 0; i < m_points.size(); i++)
	{
		max_x = std::max(m_points[i][0], max_x);
		max_y = std::max(m_points[i][1], max_y);
		min_x = std::min(m_points[i][0], min_x);
		min_y = std::min(m_points[i][1], min_y);
	}

	float norm_size = 1.0f;
	float norm_scale = norm_size / std::max(std::max(abs(max_x - min_x), abs(max_y - min_y)), 1.0f);
	float x_offset = (max_x + min_x) * norm_scale * 0.5f;
	float y_offset = (max_y + min_y) * norm_scale * 0.5f;

	// create constrainted delaunay triangulation handler
	CGAL_CDT cdt;

	// insertion
	std::vector<CGAL_Vertex_handle> vertices;
	for (int i = 0; i < m_points.size(); i++)
	{
		vertices.push_back(
			cdt.insert(CGAL_Point(m_points[i][0] * norm_scale - x_offset, m_points[i][1] * norm_scale - y_offset))
		);
	}

	for (std::size_t i = 0; i < nEdges; ++i)
	{
		unsigned int v1, v2;
		ifs >> v1 >> v2;
		cdt.insert_constraint(vertices[v1], vertices[v2]);
	}

	std::list<CGAL_Point> list_of_seeds;
	if (!ifs.eof()) {
		std::size_t nSeeds;
		ifs >> nSeeds;
		for (std::size_t i = 0; i < nSeeds; ++i)
		{
			float x1, y1;
			ifs >> x1 >> y1;
			list_of_seeds.push_back(CGAL_Point(x1 * norm_scale - x_offset, y1 * norm_scale - y_offset));
		}
	}

	ifs.close();

	std::cout << "Number of vertices: " << cdt.number_of_vertices() << std::endl;

	std::cout << "Meshing..." << std::endl;
	CGAL::refine_Delaunay_mesh_2(cdt, list_of_seeds.begin(), list_of_seeds.end(),
		CGAL_Criteria(0.125, 0.12));

	std::cout << " done." << std::endl;

	if (cdt.number_of_vertices() == 0)
		return false;

	float scale = SIZE;
	std::map<CGAL_Vertex_handle, MyMesh::VertexHandle> v_handles;
	for (auto v_it = cdt.finite_vertices_begin(); v_it != cdt.finite_vertices_end(); ++v_it)
	{
		CGAL_Vertex_handle h = v_it->handle();
		auto& p = v_it->point();
		OpenMesh::Vec3f v(p.x() * scale, 0, p.y() * scale);
		v_handles[h] = mesh.add_vertex(v);
	}

	std::vector<MyMesh::VertexHandle> face_vhandles;
	for (auto f_it = cdt.finite_faces_begin(); f_it != cdt.finite_faces_end(); ++f_it)
	{
		if (f_it->is_in_domain()) {

			CGAL_Vertex_handle h0 = f_it->vertex(0)->handle();
			CGAL_Vertex_handle h1 = f_it->vertex(1)->handle();
			CGAL_Vertex_handle h2 = f_it->vertex(2)->handle();

			face_vhandles.clear();
			face_vhandles.push_back(v_handles[h0]);
			face_vhandles.push_back(v_handles[h1]);
			face_vhandles.push_back(v_handles[h2]);

			mesh.add_face(face_vhandles);
		}
	}

	mesh.Initialization();

	return true;
}

bool GLMesh::LoadMesh(std::string fileName)
{

	OpenMesh::IO::Options opt;
	opt += OpenMesh::IO::Options::VertexTexCoord;
	if (OpenMesh::IO::read_mesh(mesh, fileName, opt))
	{
		if (!opt.check(OpenMesh::IO::Options::VertexNormal) && mesh.has_vertex_normals())
		{
			mesh.request_face_normals();
			mesh.update_normals();
			//mesh.release_face_normals();

			mesh.Initialization();

			std::string imgname = fileName.substr(0, fileName.size() - 4) + ".jpg";
			struct stat buffer;
			if (stat(imgname.c_str(), &buffer) == 0) {
				texture = new Texture2D(imgname.c_str());
			}
		}
		return true;
	}

	return false;
}

void GLMesh::LoadTexCoordToShader()
{
	if (mesh.has_vertex_texcoords2D())
	{
		std::vector<MyMesh::TexCoord2D> texCoords;
		for (MyMesh::VertexIter v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); ++v_it)
		{
			MyMesh::TexCoord2D texCoord = mesh.texcoord2D(v_it);
			texCoords.push_back(texCoord);
		}

		glBindVertexArray(this->vao.vao);

		glBindBuffer(GL_ARRAY_BUFFER, this->vao.vbo[2]);
		glBufferData(GL_ARRAY_BUFFER, sizeof(MyMesh::TexCoord2D) * texCoords.size(), &texCoords[0], GL_STATIC_DRAW);
		glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 0, 0);
		glEnableVertexAttribArray(2);

		glBindBuffer(GL_ARRAY_BUFFER, 0);
		glBindVertexArray(0);
	}
}

bool GLMesh::exportMesh(std::string filename)
{
	try
	{
		OpenMesh::IO::Options opt;
		opt += OpenMesh::IO::Options::VertexTexCoord;
		if (!OpenMesh::IO::write_mesh(mesh, filename, opt))
		{
			std::cerr << "Cannot write mesh to file : " << filename << std::endl;
			return false;
		}
		else {
			// sucess

			if (texture != nullptr) {
				std::string imgname = filename.substr(0, filename.size() - 4) + ".jpg";
				cv::imwrite(imgname,texture->GetImg());
			}

		}
	}
	catch (std::exception& x)
	{
		std::cerr << x.what() << std::endl;
		return false;
	}
	return true;
}

bool GLMesh::importPreset(std::string fname)
{
	std::ifstream ifs(fname, std::ios::binary);

	if (ifs.is_open()) {

		int N_C;
		ifs.read((char*)&N_C, sizeof(int));

		mesh.InitCompilation();

		std::vector<MyMesh::ControlPoint> controlPoints;
		for (int i = 0; i < N_C; i++) {

			MyMesh::ControlPoint cp;

			int idx;
			ifs.read((char*)&idx, sizeof(int));
			
			double w0, w1, w2;
			ifs.read((char*)&w0, sizeof(double));
			ifs.read((char*)&w1, sizeof(double));
			ifs.read((char*)&w2, sizeof(double));

			float ox, oy;
			ifs.read((char*)&ox, sizeof(float));
			ifs.read((char*)&oy, sizeof(float));


			float cx, cy;
			ifs.read((char*)&cx, sizeof(float));
			ifs.read((char*)&cy, sizeof(float));

			if (validID(idx)) {
				cp.fh = mesh.face_handle(idx);

				cp.w[0] = w0;
				cp.w[1] = w1;
				cp.w[2] = w2;

				cp.o = MyMesh::Point(ox, 0, oy);
				cp.c = MyMesh::Point(cx, 0, cy);

				controlPoints.push_back(cp);
			}
		}

		mesh.AddControlPoints(controlPoints);

		keyPoints.clear();
		keyData.clear();

		if (!ifs.eof())
		{
			int N_K;
			ifs.read((char*)&N_K, sizeof(int));
		
			for (int i = 0; i < N_K; i++) {
				float px;
				float py;

				ifs.read((char*)&px, sizeof(float));
				ifs.read((char*)&py, sizeof(float));

				keyPoints.push_back(MyMesh::Point(px, py, 0));

				std::vector<MyMesh::Point> cps;
				for (int j = 0; j < N_C; j++) {
					float cx;
					float cy;
					ifs.read((char*)&cx, sizeof(float));
					ifs.read((char*)&cy, sizeof(float));

					cps.push_back(MyMesh::Point(cx, 0, cy));
				}

				keyData.push_back(cps);
			}
		}

		mesh.Compute(0);
		UpdateShader();

		return true;
	}

	return false;
}

bool GLMesh::exportPreset(std::string fname)
{
	std::ofstream ofs(fname, std::ios::binary);

	if (ofs.is_open())
	{
		// control point counts int * 1
		int N_C = mesh.controlPoints.size();
		ofs.write((char*)&N_C, sizeof(int));

		for (int i = 0; i < N_C; i++) {
			//MyMesh::FaceHandle fh; idx -> int * 1
			//double w[3];           double * 3
			//MyMesh::Point o;       float * 2
			//MyMesh::Point c;       float * 2

			int idx = mesh.controlPoints[i].fh.idx();
			ofs.write((char*)&idx, sizeof(int));


			double w0 = mesh.controlPoints[i].w[0];
			double w1 = mesh.controlPoints[i].w[1];
			double w2 = mesh.controlPoints[i].w[2];
			ofs.write((char*)&w0, sizeof(double));
			ofs.write((char*)&w1, sizeof(double));
			ofs.write((char*)&w2, sizeof(double));


			float ox = mesh.controlPoints[i].o[0];
			float oy = mesh.controlPoints[i].o[2];
			ofs.write((char*)&ox, sizeof(float));
			ofs.write((char*)&oy, sizeof(float));


			float cx = mesh.controlPoints[i].c[0];
			float cy = mesh.controlPoints[i].c[2];
			ofs.write((char*)&cx, sizeof(float));
			ofs.write((char*)&cy, sizeof(float));
		}
		
		// keypoint counts int * 1
		int N_K = keyPoints.size();
		ofs.write((char*)&N_K, sizeof(int));

		for (int i = 0; i < N_K; i++) {
			//MyMesh::Point p float * 2

			float px = keyPoints[i][0];
			float py = keyPoints[i][2];
			ofs.write((char*)&px, sizeof(float));
			ofs.write((char*)&py, sizeof(float));

			for (int j = 0; j < N_C; j++) {
				//MyMesh::Point c float * 2
				float cx = keyData[i][j][0];
				float cy = keyData[i][j][2];
				ofs.write((char*)&cx, sizeof(float));
				ofs.write((char*)&cy, sizeof(float));
			}
		}

		return true;
	}
	return false;
}

#pragma endregion

#pragma endregion
