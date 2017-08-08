#include <fstream>
#include <thread>
#include <iostream>

#include "assimp_mesh_loader.hpp"
#include "opengl_wrapper.hpp"

std::vector<double> transpose(const std::vector<double> &transform) {
	std::vector<double> transpose(16);

	for(unsigned int i = 0; i < 16; i++) {
		unsigned int row = i / 4;
		unsigned int col = i % 4;
		transpose[i] = transform[col * 4 + row];
	}

	return transpose;
}

std::vector<std::vector<double>> getEnvironmentTriangles(const char *envFile) {
	std::vector<std::vector<double>> retTriangles;
	AssimpMeshLoader environment(envFile);

	std::vector< std::vector<AssimpMeshLoader::Vertex> > vertices;
	std::vector< std::vector<AssimpMeshLoader::Triangle> > triangles;
	std::vector< std::vector<double> > normals;

	environment.get(vertices, triangles, normals);


	const OpenGLWrapper &opengl = OpenGLWrapper::getOpenGLWrapper();
	auto color = OpenGLWrapper::Color::Red();

	std::vector<double> transform(16, 0);
	transform[0] = transform[5] = transform[10] = transform[15] = 1;

	for(unsigned int i = 0; i < triangles.size(); ++i) {
		const std::vector<AssimpMeshLoader::Vertex> &verts = vertices[i];
		const std::vector<AssimpMeshLoader::Triangle> &tris = triangles[i];
		std::vector<double> pts(84, 1.0);

		auto c = color.getColor();
		c[3] = 0.25;
		for(auto tri : tris) {
			unsigned int cur = 0;
			for(unsigned int i = 0; i < 3; ++i) {
				for(unsigned int j = 0; j < 3; ++j) {
					pts[cur++] = verts[tri[i]][j];
				}
				cur++; // add one extra for the 4th vector component
				for(unsigned int j = 0; j < 3; ++j) {
					pts[cur++] = normals[tri[i]][j];
				}
				cur++; // add one extra for the 4th vector component
				for(unsigned int j = 0; j < 4; ++j) {
					pts[cur++] = c[j];
				}
				for(unsigned int j = 0; j < 16; ++j) {
					pts[cur++] = transform[j];
				}
			}

			retTriangles.push_back(pts);
		}
	}

	return retTriangles;
}

std::vector<double> getVertFilePoints(const char *vertFile) {
	std::vector<double> retPoints;
	const OpenGLWrapper &opengl = OpenGLWrapper::getOpenGLWrapper();

	std::vector<double> point(28, 0);
	//0,1,2,3    position
	point[3] = 1;
	//4,5,6,7    normal
	point[6] = point[7] = 1;
	//8,9,10,11  color
	point[11] = 1;
	// 12-27 transform
	point[12+0] = point[12+5] = point[12+10] = point[12+15] = 1;

	std::fstream fs;
	fs.open(vertFile, std::fstream::in);

	std::vector<double> points;
	while(fs.good()) {
		fs >> point[0] >> point[1] >> point[2] >> point[8] >> point[9] >> point[10];
		points.insert(points.end(), point.begin(), point.end());
	}

	retPoints.insert(retPoints.end(), points.begin(), points.end());

	fs.close();

	return retPoints;
}

int main(int argc, char *argv[]) {
  OpenGLWrapper &opengl = OpenGLWrapper::getOpenGLWrapper(); 
  if(0) {


    double halfWidth = 15;
    std::vector<double> point(28, 0);
    //0,1,2,3    position
    point[0] = halfWidth;
    point[1] = halfWidth;
    point[3] = 1;
    //4,5,6,7    normal
    point[6] = point[7] = 1;
    //8,9,10,11  color
    point[9] = 1;
    point[11] = 1;
    // 12-27 transform
    point[12+0] = point[12+5] = point[12+10] = point[12+15] = 1;

    std::vector<double> boundingBox;
    boundingBox.insert(boundingBox.end(), point.begin(), point.end());

    point[0] = -halfWidth;
    point[1] = halfWidth;
    boundingBox.insert(boundingBox.end(), point.begin(), point.end());
    point[0] = -halfWidth;
    point[1] = -halfWidth;
    boundingBox.insert(boundingBox.end(), point.begin(), point.end());
    point[0] = halfWidth;
    point[1] = -halfWidth;
    boundingBox.insert(boundingBox.end(), point.begin(), point.end());
    point[0] = halfWidth;
    point[1] = halfWidth;
    boundingBox.insert(boundingBox.end(), point.begin(), point.end());

    auto triangles = getEnvironmentTriangles(argv[1]);     

    auto triangles2 = getEnvironmentTriangles("/home/guty/gopath/src/github.com/skiesel/moremotionplanning/models/car2_planar_robot_SCALED.dae");

    //OpenGLWrapper &opengl = OpenGLWrapper::getOpenGLWrapper();

    // opengl.zoom(1, 10);
    auto lambda = [&]() {

      opengl.drawLines(boundingBox);

      for(const auto &e : triangles) {
        opengl.drawTriangles(e);
      }

      std::vector<double> translate(16);
      translate[0] = translate[5] = translate[10] = translate[15] = 1;

      translate[3] = -5;
      translate[7] = -5;
      translate[11] = 0;

      auto transposed = transpose(translate);

      for(const auto &e : triangles2) {
        opengl.drawTriangles(e);
        // auto tri = e;
        // for(unsigned int j = 0; j < tri.size(); j+=28) {
        // 	for(unsigned int k = 0; k < 16; k++) {
        // 		tri[j + 12 + k] = transposed[k];
        // 	}
        // }
        // opengl.drawTriangles(tri);
      }

      glPointSize(2);

      std::vector<double> point(28, 0);
      //0,1,2,3    position
      point[0] = -10;
      point[1] = 13;
      point[2] = 0;
      point[3] = 1;
      //4,5,6,7    normal
      point[6] = point[7] = 1;
      //8,9,10,11  color
      point[9] = 1;
      point[11] = 1;
      // 12-27 transform
      point[12+0] = point[12+5] = point[12+10] = point[12+15] = 1;

      opengl.drawPoints(point);

      point[0] = -10;
      point[1] = -13;
      point[2] = 0;
      opengl.drawPoints(point);
		

      point[0] = -13;
      point[1] = 13;
      point[2] = 0;
      opengl.drawPoints(point);

      point[0] = -10;
      point[1] = -11;
      point[2] = 0;
      opengl.drawPoints(point);


      point[0] = -12;
      point[1] = 13;
      point[2] = 0;
      opengl.drawPoints(point);

      point[0] = -13;
      point[1] = -12;
      point[2] = 0;
      opengl.drawPoints(point);


      point[0] = -13.5;
      point[1] = 13.5;
      point[2] = 0;
      opengl.drawPoints(point);

      point[0] = -13.5;
      point[1] = -13.5;
      point[2] = 0;
      opengl.drawPoints(point);


      point[0] = -12;
      point[1] = 12;
      point[2] = 0;
      opengl.drawPoints(point);

      point[0] = -12;
      point[1] = -10;
      point[2] = 0;
      opengl.drawPoints(point);



    };
    OpenGLWrapper::getOpenGLWrapper().runWithCallback(lambda);
  };
  if(1) {
    //OpenGLWrapper &opengl = OpenGLWrapper::getOpenGLWrapper();

    std::vector<double> point(28, 0);
    //0,1,2,3    position
    point[3] = 1;
    //4,5,6,7    normal
    point[6] = point[7] = 1;
    //8,9,10,11  color
    point[8] = 1;
    point[11] = 1;
    // 12-27 transform
    point[12+0] = point[12+5] = point[12+10] = point[12+15] = 1;

    unsigned int current = 1;
    bool first = true;
    std::chrono::milliseconds framerate(250);
    std::chrono::time_point<std::chrono::system_clock> start, end;
    auto lambda = [&]() {
      std::chrono::duration<double, std::milli> elapsedms = end-start;
      start = std::chrono::system_clock::now();
      glPointSize(1);

      fprintf(stderr, "current: %s\n", argv[2]);
      if(!first) {
        std::this_thread::sleep_for(framerate - elapsedms);
      } else {
        first = false;
      }
      std::fstream fs;
      fs.open(argv[2], std::fstream::in);

      std::vector<double> points;
      std::string ignore;
      unsigned int counter = 0;
      while(fs.good()) {
        if(counter > 100) {
          counter = 0;
          opengl.drawPoints(points);
          points.clear();
        }
        fs >> ignore >> point[0] >> point[1] >> point[2] >> point[8] >> point[9] >> point[10] >> point[11];
 
        points.insert(points.end(), point.begin(), point.end());
        counter++;
      }

      opengl.drawPoints(points);

      fs.close();


      end = std::chrono::system_clock::now();


    };
    opengl.runWithCallback(lambda);

  };
  return 0;
}
