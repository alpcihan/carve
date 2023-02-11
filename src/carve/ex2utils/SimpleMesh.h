#pragma once

#ifndef SIMPLE_MESH_H
#define SIMPLE_MESH_H

#include <iostream>
#include <fstream>

#include "Eigen.h"
#include <Eigen/Core> 
#include <Eigen/Dense>

#include <pcl/surface/poisson.h>

#include <pcl/point_types.h>
#include <pcl/io/obj_io.h>

typedef Eigen::Vector3f Vertex;

struct Triangle
{
	unsigned int idx0;
	unsigned int idx1;
	unsigned int idx2;
	Triangle(unsigned int _idx0, unsigned int _idx1, unsigned int _idx2) :
		idx0(_idx0), idx1(_idx1), idx2(_idx2)
	{}
};

class SimpleMesh
{
public:

	void Clear()
	{
		m_vertices.clear();
		m_triangles.clear();
	}

	unsigned int AddVertex(Vertex& vertex)
	{
		unsigned int vId = (unsigned int)m_vertices.size();
		m_vertices.push_back(vertex);
		return vId;
	}

	unsigned int AddFace(unsigned int idx0, unsigned int idx1, unsigned int idx2)
	{
		unsigned int fId = (unsigned int)m_triangles.size();
		Triangle triangle(idx0, idx1, idx2);
		m_triangles.push_back(triangle);
		return fId;
	}

	std::vector<Vertex>& GetVertices()
	{
		return m_vertices;
	}

	std::vector<Triangle>& GetTriangles()
	{
		return m_triangles;
	}

	bool WriteMesh(const std::string& filename)
	{
		// Write off file
		std::ofstream outFile(filename);
		if (!outFile.is_open()) return false;

		// write header
		outFile << "OFF" << std::endl;
		outFile << m_vertices.size() << " " << m_triangles.size() << " 0" << std::endl;

		// save vertices
		for (unsigned int i = 0; i<m_vertices.size(); i++)
		{
			outFile << m_vertices[i].x() << " " << m_vertices[i].y() << " " << m_vertices[i].z() << std::endl;
		}

		// save faces
		for (unsigned int i = 0; i<m_triangles.size(); i++)
		{
			outFile << "3 " << m_triangles[i].idx0 << " " << m_triangles[i].idx1 << " " << m_triangles[i].idx2 << std::endl;
		}

		// close file
		outFile.close();

		return true;
	}

private:
	std::vector<Vertex> m_vertices;
	std::vector<Triangle> m_triangles;
};

class PointCloud
{
public:
	bool ReadFromFile(const std::string& filename)
	{
		std::ifstream is(filename, std::ios::in | std::ios::binary);
		if (!is.is_open())
		{
			std::cout << "ERROR: unable to read input file!" << std::endl;
			return false;
		}

		char nBytes;
		is.read(&nBytes, sizeof(char));

		unsigned int n;
		is.read((char*)&n, sizeof(unsigned int));

		if (nBytes == sizeof(float))
		{
			float* ps = new float[3 * n];

			is.read((char*)ps, 3 * sizeof(float)*n);

			for (unsigned int i = 0; i < n; i++)
			{
				Eigen::Vector3f p(ps[3 * i + 0], ps[3 * i + 1], ps[3 * i + 2]);
				m_points.push_back(p);
			}

			is.read((char*)ps, 3 * sizeof(float)*n);
			for (unsigned int i = 0; i < n; i++)
			{
				Eigen::Vector3f p(ps[3 * i + 0], ps[3 * i + 1], ps[3 * i + 2]);
				m_normals.push_back(p);
			}

			delete ps;
		}
		else
		{
			double* ps = new double[3 * n];

			is.read((char*)ps, 3 * sizeof(double)*n);

			for (unsigned int i = 0; i < n; i++)
			{
				Eigen::Vector3f p((float)ps[3 * i + 0], (float)ps[3 * i + 1], (float)ps[3 * i + 2]);
				m_points.push_back(p);
			}

			is.read((char*)ps, 3 * sizeof(double)*n);

			for (unsigned int i = 0; i < n; i++)
			{
				Eigen::Vector3f p((float)ps[3 * i + 0], (float)ps[3 * i + 1], (float)ps[3 * i + 2]);
				m_normals.push_back(p);
			}

			delete ps;
		}


		//std::ofstream file("pointcloud.off");
		//file << "OFF" << std::endl;
		//file << m_points.size() << " 0 0" << std::endl;
		//for(unsigned int i=0; i<m_points.size(); ++i)
		//	file << m_points[i].x() << " " << m_points[i].y() << " " << m_points[i].z() << std::endl;
		//file.close();

		return true;
	}

	bool ReadFromSpace(std::vector<bool>& in_space){
		//////// this reads carved volume deletes useless data and calculates normals

		//store points
		space = in_space;
        size = (uint32_t)std::cbrt(space.size()); //maybe pass it?

        for (int i = 1; i < size-1; i++)
        {
            for (int j = 1; j < size-1; j++)
            {
                for (int k = 1; k < size-1; k++)
                {
					uint32_t i1D = (k * size + j) * size + i;


					if (!space[i1D] || numberOfNeighbors(i,j,k)>18 || numberOfNeighbors(i,j,k)==1 )
					{
						continue;
					}

					Eigen::Vector3f p((float)i/size,(float)j/size,(float)k/size);
					m_points.push_back(p);
					m_normals.push_back(roughNormal(i,j,k).normalized());


                }
            }
        }

		CRV_INFO("POINTS:::::"<<m_points.size());


		//calculate normals
		// number of closest points for normal estimation can be a parameter

		unsigned int nNb = 6; 
		unsigned int idx[nNb];

		

		
		CRV_INFO("ESTIMATING NORMALS")
		CRV_INFO("Writing carved point cloud into \"output_wn.ply\"...");
		std::string filename = "output_wn.ply";
		std::ofstream file(CRV_RELATIVE(filename));
        file << "ply\nformat ascii 1.0\nelement vertex " << m_points.size() << "\nproperty float x\nproperty float y\nproperty float z\nproperty float nx\nproperty float ny\nproperty float nz\nend_header\n";


		for (int i=0;i<m_points.size();i++) {
			Eigen::Vector3f point = m_points.at(i);
			uint nb_within_range = GetnClosestPoints(point,idx,0.02,nNb);

/*  
			// this part is not working properly I replaced it with a snippet from github, I also lost the the source of this.
			// https://gist.github.com/ialhashim/0a2554076a6cf32831ca
  			Eigen::MatrixXf pointsMatrix(3,nb_within_range);
			for (int nb=0;nb<nb_within_range;nb++) {
				pointsMatrix.block(0,i,3,1) = m_points.at(idx[nb]);  
			}
			JacobiSVD<MatrixXf> svd(pointsMatrix,ComputeThinU | ComputeThinV);
			std::vector<float> v;
			v.push_back(svd.singularValues()(0));
			v.push_back(svd.singularValues()(1));
			v.push_back(svd.singularValues()(2));
			Eigen::Vector3f normal =  svd.matrixU().block(0,std::distance(std::begin(v), std::min_element(std::begin(v), std::end(v))),3,1);
*/


			Eigen::Matrix< Eigen::Vector3f::Scalar, Eigen::Dynamic, Eigen::Dynamic > coord(3, nb_within_range);
			for (int nb=0;nb<nb_within_range;nb++) {
				coord.col(nb) = m_points.at(idx[nb]);  
			}
			Eigen::Vector3f centroid(coord.row(0).mean(), coord.row(1).mean(), coord.row(2).mean());
			coord.row(0).array() -= centroid(0); coord.row(1).array() -= centroid(1); coord.row(2).array() -= centroid(2);
			auto svd = coord.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
			Eigen::Vector3f normal = svd.matrixU().rightCols(1).normalized();


			m_normals.at(i) = (((m_normals.at(i).dot(normal))/(m_normals.at(i).norm())) * normal ).normalized();
			
			
		}

		pcl::PointCloud<pcl::PointNormal>::Ptr cloud (new pcl::PointCloud<pcl::PointNormal> ());
		std::cout << sizeof(pcl::PointNormal) << std::endl;
		cloud->width    = m_points.size();
		cloud->height   = 1;
		cloud->is_dense = false;
		cloud->points.resize (cloud->width * cloud->height);

		for (int i=0;i<m_points.size();i++) {
			if (m_normals.at(i).norm() == 0) {
				unsigned int idc[6];
				GetnClosestPoints(m_points.at(i),idc,0.1, 6);
				m_normals.at(i) = (m_normals.at(idc[0]) + m_normals.at(idc[1]) + m_normals.at(idc[2]) + m_normals.at(idc[3]) + m_normals.at(idc[4]) + m_normals.at(idc[5])  /6).normalized();
				}
			file << m_points.at(i).x() << " " << m_points.at(i).y() << " " << m_points.at(i).z() << " " << m_normals.at(i).x() << " " << m_normals.at(i).y() << " " << m_normals.at(i).z() << "\n";
			pcl::PointNormal p = pcl::PointNormal(m_points.at(i).x(),m_points.at(i).y(),m_points.at(i).z(),m_normals.at(i).x(), m_normals.at(i).y(),m_normals.at(i).z() ,0.f);
			cloud->push_back(p);
		}

		pcl::Poisson<pcl::PointNormal> poisson;
		poisson.setInputCloud(cloud);
		pcl::PolygonMesh recMesh;
		poisson.reconstruct(recMesh);
		filename = "poiss_mesh.obj";
		pcl::io::saveOBJFile ( CRV_RELATIVE(filename),recMesh,5); 		

		CRV_INFO("ESTIMATION DONE")
        CRV_INFO("Saved the carved point cloud");
        file.close();

		return true;
	}

	std::vector<Eigen::Vector3f>& GetPoints()
	{
		return m_points;
	}

	std::vector<Eigen::Vector3f>& GetNormals()
	{
		return m_normals;
	}

	uint numberOfNeighbors(int i, int j, int k) {

		uint sum =0;
		for(int xx=-1;xx<2;xx++) {
			if (i==0 || i==size-1) continue;
			for(int yy=-1;yy<2;yy++) {
				if (j==0 || j==size-1) continue;
				for(int zz=-1;zz<2;zz++) {
					if (k==0 || k==size-1) continue;
					sum+=space[((k+zz) * size + j+yy) * size + i+xx];
				}
			}
		}
		return sum;
	}






	Eigen::Vector3f roughNormal(int i, int j, int k) {

		Eigen::Vector3f maxVec;
		Eigen::Vector3f minVec;
		uint maxNeighbours=0;
		uint minNeighbours=28;
		uint nNb = 0;

		for(int xx=-1;xx<2;xx++) {
			if (i==0 || i==size-1) continue;
			for(int yy=-1;yy<2;yy++) {
				if (j==0 || j==size-1) continue;
				for(int zz=-1;zz<2;zz++) {
					if (k==0 || k==size-1) continue;
					nNb = numberOfNeighbors(i+xx,j+yy,k+zz);
					if (nNb<minNeighbours) { minVec = Eigen::Vector3f(i+xx,j+yy,k+zz); minNeighbours = nNb; }
					if (nNb>maxNeighbours) { maxVec = Eigen::Vector3f(i+xx,j+yy,k+zz); maxNeighbours = nNb; }
				}
			}
		}
		return minVec-maxVec;
	}


	unsigned int GetClosestPoint(Eigen::Vector3f& p)
	{
		unsigned int idx = 0;

		float min_dist = std::numeric_limits<float>::max();
		for (unsigned int i = 0; i < m_points.size(); ++i)
		{
			float dist = (p - m_points[i]).norm();
			if (min_dist > dist)
			{
				idx = i;
				min_dist = dist;
			}
		}

		return idx;
	}


	uint GetnClosestPoints(Eigen::Vector3f& p,unsigned int *idx,float maxDist,uint maxNB)
	{

		uint numba = 0;
		for (unsigned int i = 0; i < m_points.size(); ++i)
		{
			float dist = (p - m_points[i]).norm();
			if (dist<maxDist)
			{
				idx[numba] = i;
				numba++;
			}
			if (numba == maxNB) break;
		}
		return numba;
	}



private:
	std::vector<Eigen::Vector3f> m_points;
	std::vector<Eigen::Vector3f> m_normals;
	int size;
	std::vector<bool> space;

};

#endif // SIMPLE_MESH_H
