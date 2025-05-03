/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2025, Yafei Ou and Mahdi Tavakoli
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef CR_TEST_OBJ_LOADER_H
#define CR_TEST_OBJ_LOADER_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <stdexcept>

#include "vec3.h"

class ObjLoader
{
private:
    std::vector<crmpm::Vec3f3> vertices;
    std::vector<unsigned int> triangles;

    // Helper function to parse face indices
    unsigned int parseIndex(const std::string &token)
    {
        size_t slashPos = token.find('/');
        if (slashPos != std::string::npos)
        {
            return std::stoi(token.substr(0, slashPos)) - 1; // Convert 1-based to 0-based
        }
        return std::stoi(token) - 1; // Fallback for cases without slashes
    }

public:
    // Loads an OBJ file
    void load(const std::string &filename)
    {
        std::ifstream file(filename);
        if (!file.is_open())
        {
            throw std::runtime_error("Could not open file: " + filename);
        }

        std::string line;
        while (std::getline(file, line))
        {
            std::istringstream lineStream(line);
            std::string prefix;
            lineStream >> prefix;

            if (prefix == "v")
            {
                // Vertex line
                float x, y, z;
                lineStream >> x >> y >> z;
                vertices.emplace_back(x, y, z);
            }
            else if (prefix == "f")
            {
                std::string vertex1, vertex2, vertex3;
                lineStream >> vertex1 >> vertex2 >> vertex3;
                triangles.push_back(parseIndex(vertex1));
                triangles.push_back(parseIndex(vertex2));
                triangles.push_back(parseIndex(vertex3));
            }
        }

        file.close();
    }

    // Returns the array of vertices
    std::vector<crmpm::Vec3f3> &getVertices()
    {
        return vertices;
    }

    // Returns the array of triangles
    std::vector<unsigned int> &getTriangles()
    {
        return triangles;
    }
};

#endif // !CR_TEST_OBJ_LOADER_H