#pragma once

#include <string>
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <glad/glad.h>
#include <queue>
#include <Eigen/Dense>
#include <Eigen/Sparse>

#include "BufferObject.h"
#include "Texture.h"

class serverController;

typedef OpenMesh::TriMesh_ArrayKernelT<>  TriMesh;


#define SELECT_TYPE_CONTROL_POINT 0
#define SELECT_TYPE_WEIGHT 1

class MyMesh : public TriMesh
{
public:
	MyMesh();
	~MyMesh();

	void Reset();

	void Initialization();
	void Registration();
	void preComputeG();
	void preComputeL1();
	void preComputeL2();

	struct ControlPoint {
		MyMesh::FaceHandle fh;
		double w[3];
		MyMesh::Point o;
		MyMesh::Point c;
	};

	void select(unsigned int, MyMesh::Point);

	void InitCompilation();
	void AddControlPoint(ControlPoint);
	void AddControlPoints(std::vector<ControlPoint>&);
	void RemoveControlPoint(unsigned int);
	void Compilation();

	void SetEdgeWeights(std::set<unsigned int>&);

	void Compute(unsigned int);
	void Step1();
	void Step2();

	double W = 1000;

	std::vector<ControlPoint> controlPoints;
	std::vector<MyMesh::Point> deformed_vertices;

private:
	bool holdCompilation = false;

	Eigen::SparseMatrix<double> L1, L2, LL1, LL2;
	Eigen::SparseMatrix<double> C1, C2, CC1, CC2;

	std::vector<Eigen::Triplet<double>> C1_triplets;
	std::vector<Eigen::Triplet<double>> C2_triplets;

	Eigen::SparseMatrix<double, Eigen::RowMajor> A1, A2;
	Eigen::SparseMatrix<double> AA1, AA2;
	Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver1, solver2;

	Eigen::MatrixXd b1, b2x, b2y;
	Eigen::VectorXd V1, V2x, V2y;

	OpenMesh::EPropHandleT<Eigen::MatrixXd> prop_G;
	OpenMesh::EPropHandleT<double> prop_W;
	OpenMesh::EPropHandleT<double> prop_W_C;

};

class GLMesh
{
public:
	GLMesh();
	~GLMesh();

	float SIZE = 250;

	bool Init(std::string fileName);
	void resetMesh();

	// i / o
	bool exportMesh(std::string fileName);
	// control point io
	bool importPreset(std::string fname);
	bool exportPreset(std::string fname);

	// rendering
	void renderMesh();
	void renderSelectedMesh();
	void renderControlPoints();
	void renderKeyPoints();

	// select triangle and add constraint
	void select(unsigned int, MyMesh::Point);
	void selectTri(unsigned int, bool);

	void applyTriangleWeights();

	// control points control
	unsigned int select_id = -1;
	void selectControlPoint(MyMesh::Point);
	void dragControlPoint(MyMesh::Point);
	void remove_selected();

	// keypoints control
	unsigned int select_k_id = -1;
	MyMesh::Point current_key;
	void addKeyPoint(MyMesh::Point);
	void selectKeyPoint(MyMesh::Point);
	void dragKeyPoint(MyMesh::Point);
	void removeKeyPoint();
	void Interpolate();

	// connetion
	bool is_changed[1] = { false };
	bool is_decoding = false;
	void socketCallback(char* buffer, int length);
	void checkUpdate();

	// utilities
	bool validID(unsigned int);

private:
	MyMesh mesh;
	VAO vao;
	Texture2D* texture = nullptr;

	serverController* sc;

	std::vector<std::vector<MyMesh::Point>> keyData;
	std::vector<MyMesh::Point> keyPoints;

	std::set<unsigned int> constrainedTriIDs;
	bool edge_weight_modified = false;

	bool Load2DImage(std::string fileName);
	bool Load2DModel(std::string fileName);
	bool LoadMesh(std::string fileName);
	void LoadToShader();
	void LoadToShader(
		std::vector<MyMesh::Point>& vertices,
		std::vector<MyMesh::Normal>& normals,
		std::vector<unsigned int>& indices);
	void UpdateShader();
	void LoadTexCoordToShader();
};

