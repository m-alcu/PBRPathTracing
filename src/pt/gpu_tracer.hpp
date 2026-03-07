#pragma once
#ifndef GL_GLEXT_PROTOTYPES
#define GL_GLEXT_PROTOTYPES
#endif
#include <GL/gl.h>
#include <GL/glext.h>

#include "scene.hpp"
#include "camera.hpp"
#include "tracer.hpp"   // BRDFMode
#include "torus.hpp"
#include "box.hpp"
#include "capsule.hpp"
#include "cylinder.hpp"
#include "rounded_box.hpp"

#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>

// ---------------------------------------------------------------------------
// GPU-side packed structs (std430 aligned)
// ---------------------------------------------------------------------------

struct GPUTriangle {
    // UVs packed into w-components to match shader layout (std430, 7 vec4s)
    float v0[3]; float matId;    // vec4 v0_matId
    float v1[3]; float u0;       // vec4 v1_u0      (u0 = uv[0].u)
    float v2[3]; float v0uv;     // vec4 v2_v0      (v0uv = uv[0].v)
    float n0[3]; float u1;       // vec4 n0_u1      (u1 = uv[1].u)
    float n1[3]; float v1uv;     // vec4 n1_v1      (v1uv = uv[1].v)
    float n2[3]; float u2;       // vec4 n2_u2      (u2 = uv[2].u)
    float v2uv;  float pad[3];   // vec4 uv2v_pad   (v2uv = uv[2].v)
};

struct GPUSphere {
    float cx, cy, cz, radius;
    int   matId;
    int   pad0, pad1, pad2;
};

struct GPUPlane {
    float nx, ny, nz, offset;  // vec4 normalOffset
    int   matId;
    int   pad0, pad1, pad2;
};

struct GPUTorus {
    float cx,cy,cz,majorR;   // vec4 centerR
    float ax,ay,az,minorR;   // vec4 axisMin
    int   matId; int pad0,pad1,pad2;
};

struct GPUBox {
    float cx,cy,cz,pad0_;   // vec4 center
    float hx,hy,hz,pad1_;   // vec4 half
    int   matId; int pad0,pad1,pad2;
};

struct GPUCapsule {
    float ax,ay,az,radius;  // vec4 aR
    float bx,by,bz,pad_;   // vec4 b
    int   matId; int pad0,pad1,pad2;
};

struct GPUCylinder {
    float cx,cy,cz,radius;      // vec4 centerR
    float ax,ay,az,halfHeight;  // vec4 axisH
    int   matId; int pad0,pad1,pad2;
};

struct GPURoundedBox {
    float cx,cy,cz,cornerRadius; // vec4 centerCR
    float hx,hy,hz,pad_;         // vec4 half
    int   matId; int pad0,pad1,pad2;
};

struct GPUBVHNode {
    float mnX, mnY, mnZ; float left;   // left child index (or triStart)
    float mxX, mxY, mxZ; float right;  // right child index (or triCount encoded elsewhere)
    int   triStart;
    int   triCount;
    int   pad0, pad1;
};

struct GPUMaterial {
    float albedo[3]; float metallic;
    float emission[3]; float roughness;
    float ior;
    float trans;
    int   albedoTex;  // -1 = no texture, else index into scene textures
    float pad;
};

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static std::string readFile(const std::string& path) {
    std::ifstream f(path);
    if (!f) { std::fprintf(stderr, "GPUTracer: cannot open '%s'\n", path.c_str()); return ""; }
    std::ostringstream ss; ss << f.rdbuf(); return ss.str();
}

static GLuint compileShader(GLenum type, const std::string& src, const char* name) {
    GLuint s = glCreateShader(type);
    const char* p = src.c_str();
    glShaderSource(s, 1, &p, nullptr);
    glCompileShader(s);
    GLint ok; glGetShaderiv(s, GL_COMPILE_STATUS, &ok);
    if (!ok) {
        char log[4096]; glGetShaderInfoLog(s, sizeof(log), nullptr, log);
        std::fprintf(stderr, "Shader compile error (%s):\n%s\n", name, log);
    }
    return s;
}

static GLuint linkProgram(std::initializer_list<GLuint> shaders, const char* name) {
    GLuint p = glCreateProgram();
    for (GLuint s : shaders) glAttachShader(p, s);
    glLinkProgram(p);
    GLint ok; glGetProgramiv(p, GL_LINK_STATUS, &ok);
    if (!ok) {
        char log[4096]; glGetProgramInfoLog(p, sizeof(log), nullptr, log);
        std::fprintf(stderr, "Program link error (%s):\n%s\n", name, log);
    }
    for (GLuint s : shaders) { glDetachShader(p, s); glDeleteShader(s); }
    return p;
}

// ---------------------------------------------------------------------------
// GPUPathTracer
// ---------------------------------------------------------------------------

class GPUPathTracer {
public:
    int W = 0, H = 0;

    bool init(int w, int h, const std::string& shaderDir) {
        W = w; H = h;

        // --- Compute program ---
        std::string compSrc = readFile(shaderDir + "/pathtracer.comp");
        if (compSrc.empty()) return false;
        GLuint comp = compileShader(GL_COMPUTE_SHADER, compSrc, "pathtracer.comp");
        mComputeProg = linkProgram({comp}, "compute");

        // --- Display program ---
        std::string vSrc = readFile(shaderDir + "/display.vert");
        std::string fSrc = readFile(shaderDir + "/display.frag");
        if (vSrc.empty() || fSrc.empty()) return false;
        GLuint vs = compileShader(GL_VERTEX_SHADER,   vSrc, "display.vert");
        GLuint fs = compileShader(GL_FRAGMENT_SHADER, fSrc, "display.frag");
        mDisplayProg = linkProgram({vs, fs}, "display");

        // --- Accumulation texture (RGBA32F) ---
        glGenTextures(1, &mAccumTex);
        glBindTexture(GL_TEXTURE_2D, mAccumTex);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, W, H, 0, GL_RGBA, GL_FLOAT, nullptr);
        glBindTexture(GL_TEXTURE_2D, 0);

        // --- FBO for clearing the accumulation texture ---
        glGenFramebuffers(1, &mClearFBO);
        glBindFramebuffer(GL_FRAMEBUFFER, mClearFBO);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
                               GL_TEXTURE_2D, mAccumTex, 0);
        glBindFramebuffer(GL_FRAMEBUFFER, 0);

        // --- Empty VAO for display fullscreen triangle ---
        glGenVertexArrays(1, &mVAO);

        reset();
        return true;
    }

    void uploadScene(const PBRScene& scene) {
        deleteSSBOs();
        deleteTextures();

        // Triangles (with UVs packed in w-slots)
        {
            std::vector<GPUTriangle> tris;
            tris.reserve(scene.triangles.size());
            for (const auto& t : scene.triangles) {
                GPUTriangle g;
                g.v0[0]=t.v[0].x; g.v0[1]=t.v[0].y; g.v0[2]=t.v[0].z;
                g.matId=(float)t.matId;
                g.v1[0]=t.v[1].x; g.v1[1]=t.v[1].y; g.v1[2]=t.v[1].z;
                g.u0   =t.uv[0][0];
                g.v2[0]=t.v[2].x; g.v2[1]=t.v[2].y; g.v2[2]=t.v[2].z;
                g.v0uv =t.uv[0][1];
                g.n0[0]=t.n[0].x; g.n0[1]=t.n[0].y; g.n0[2]=t.n[0].z;
                g.u1   =t.uv[1][0];
                g.n1[0]=t.n[1].x; g.n1[1]=t.n[1].y; g.n1[2]=t.n[1].z;
                g.v1uv =t.uv[1][1];
                g.n2[0]=t.n[2].x; g.n2[1]=t.n[2].y; g.n2[2]=t.n[2].z;
                g.u2   =t.uv[2][0];
                g.v2uv =t.uv[2][1];
                g.pad[0]=g.pad[1]=g.pad[2]=0;
                tris.push_back(g);
            }
            mSSBO[0] = makeSSBO(tris.data(), tris.size() * sizeof(GPUTriangle));
        }

        // TriIndices
        mSSBO[1] = makeSSBO(scene.triIndices.data(),
                            scene.triIndices.size() * sizeof(int));

        // BVH nodes
        {
            std::vector<GPUBVHNode> nodes;
            nodes.reserve(scene.bvh.size());
            for (const auto& n : scene.bvh) {
                GPUBVHNode g;
                g.mnX=n.aabb.mn.x; g.mnY=n.aabb.mn.y; g.mnZ=n.aabb.mn.z;
                g.left =(float)n.left;
                g.mxX=n.aabb.mx.x; g.mxY=n.aabb.mx.y; g.mxZ=n.aabb.mx.z;
                g.right=(float)n.right;
                g.triStart=n.triStart; g.triCount=n.triCount;
                g.pad0=0; g.pad1=0;
                nodes.push_back(g);
            }
            mSSBO[2] = makeSSBO(nodes.data(), nodes.size() * sizeof(GPUBVHNode));
        }

        // Spheres
        {
            std::vector<GPUSphere> sphs;
            sphs.reserve(scene.spheres.size());
            for (const auto& s : scene.spheres) {
                GPUSphere g;
                g.cx=s.center.x; g.cy=s.center.y; g.cz=s.center.z;
                g.radius=s.radius; g.matId=s.matId;
                g.pad0=g.pad1=g.pad2=0;
                sphs.push_back(g);
            }
            mSSBO[3] = makeSSBO(sphs.data(), sphs.size() * sizeof(GPUSphere));
        }

        // Materials
        {
            std::vector<GPUMaterial> mats;
            mats.reserve(scene.materials.size());
            for (const auto& m : scene.materials) {
                GPUMaterial g;
                g.albedo[0]=m.albedo.x; g.albedo[1]=m.albedo.y; g.albedo[2]=m.albedo.z;
                g.metallic=m.metallic;
                g.emission[0]=m.emission.x; g.emission[1]=m.emission.y; g.emission[2]=m.emission.z;
                g.roughness=m.roughness;
                g.ior=m.ior; g.trans=m.transmission;
                g.albedoTex=m.albedoTex;
                g.pad=0;
                mats.push_back(g);
            }
            mSSBO[4] = makeSSBO(mats.data(), mats.size() * sizeof(GPUMaterial));
        }

        // Upload scene textures as GL_TEXTURE_2D objects
        for (const auto& tex : scene.textures) {
            GLuint glTex = 0;
            glGenTextures(1, &glTex);
            glBindTexture(GL_TEXTURE_2D, glTex);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
            if (tex.isValid()) {
                glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, tex.w, tex.h, 0,
                             GL_RGBA, GL_UNSIGNED_BYTE, tex.data.data());
                glGenerateMipmap(GL_TEXTURE_2D);
            }
            glBindTexture(GL_TEXTURE_2D, 0);
            mTextures.push_back(glTex);
        }

        // Light sphere indices
        mSSBO[5] = makeSSBO(scene.lightSphereIndices.data(),
                            scene.lightSphereIndices.size() * sizeof(int));

        // Planes (binding 7)
        {
            std::vector<GPUPlane> plns;
            plns.reserve(scene.planes.size());
            for (const auto& p : scene.planes) {
                GPUPlane g;
                g.nx=p.normal.x; g.ny=p.normal.y; g.nz=p.normal.z;
                g.offset=p.offset; g.matId=p.matId;
                g.pad0=g.pad1=g.pad2=0;
                plns.push_back(g);
            }
            mSSBO[6] = makeSSBO(plns.data(), plns.size() * sizeof(GPUPlane));
        }

        // Tori (binding 8)
        {
            std::vector<GPUTorus> v; v.reserve(scene.tori.size());
            for (const auto& o : scene.tori) {
                GPUTorus g;
                g.cx=o.center.x; g.cy=o.center.y; g.cz=o.center.z; g.majorR=o.majorR;
                g.ax=o.axis.x;   g.ay=o.axis.y;   g.az=o.axis.z;   g.minorR=o.minorR;
                g.matId=o.matId; g.pad0=g.pad1=g.pad2=0; v.push_back(g);
            }
            mSSBO[7] = makeSSBO(v.data(), v.size() * sizeof(GPUTorus));
        }
        // Boxes (binding 9)
        {
            std::vector<GPUBox> v; v.reserve(scene.boxes.size());
            for (const auto& o : scene.boxes) {
                GPUBox g;
                g.cx=o.center.x; g.cy=o.center.y; g.cz=o.center.z; g.pad0_=0;
                g.hx=o.half.x;   g.hy=o.half.y;   g.hz=o.half.z;   g.pad1_=0;
                g.matId=o.matId; g.pad0=g.pad1=g.pad2=0; v.push_back(g);
            }
            mSSBO[8] = makeSSBO(v.data(), v.size() * sizeof(GPUBox));
        }
        // Capsules (binding 10)
        {
            std::vector<GPUCapsule> v; v.reserve(scene.capsules.size());
            for (const auto& o : scene.capsules) {
                GPUCapsule g;
                g.ax=o.a.x; g.ay=o.a.y; g.az=o.a.z; g.radius=o.radius;
                g.bx=o.b.x; g.by=o.b.y; g.bz=o.b.z; g.pad_=0;
                g.matId=o.matId; g.pad0=g.pad1=g.pad2=0; v.push_back(g);
            }
            mSSBO[9] = makeSSBO(v.data(), v.size() * sizeof(GPUCapsule));
        }
        // Cylinders (binding 11)
        {
            std::vector<GPUCylinder> v; v.reserve(scene.cylinders.size());
            for (const auto& o : scene.cylinders) {
                GPUCylinder g;
                g.cx=o.center.x; g.cy=o.center.y; g.cz=o.center.z; g.radius=o.radius;
                g.ax=o.axis.x;   g.ay=o.axis.y;   g.az=o.axis.z;   g.halfHeight=o.halfHeight;
                g.matId=o.matId; g.pad0=g.pad1=g.pad2=0; v.push_back(g);
            }
            mSSBO[10] = makeSSBO(v.data(), v.size() * sizeof(GPUCylinder));
        }
        // RoundedBoxes (binding 12)
        {
            std::vector<GPURoundedBox> v; v.reserve(scene.roundedBoxes.size());
            for (const auto& o : scene.roundedBoxes) {
                GPURoundedBox g;
                g.cx=o.center.x; g.cy=o.center.y; g.cz=o.center.z; g.cornerRadius=o.cornerRadius;
                g.hx=o.half.x;   g.hy=o.half.y;   g.hz=o.half.z;   g.pad_=0;
                g.matId=o.matId; g.pad0=g.pad1=g.pad2=0; v.push_back(g);
            }
            mSSBO[11] = makeSSBO(v.data(), v.size() * sizeof(GPURoundedBox));
        }

        mNumSpheres     = (int)scene.spheres.size();
        mNumBVHNodes    = (int)scene.bvh.size();
        mNumLights      = (int)scene.lightSphereIndices.size();
        mNumPlanes      = (int)scene.planes.size();
        mNumTori        = (int)scene.tori.size();
        mNumBoxes       = (int)scene.boxes.size();
        mNumCapsules    = (int)scene.capsules.size();
        mNumCylinders   = (int)scene.cylinders.size();
        mNumRoundedBoxes= (int)scene.roundedBoxes.size();

        // Ensure SSBOs have at least 1 element to avoid zero-size buffers
        if (!scene.triangles.empty() && scene.triIndices.empty()) {
            // shouldn't happen but guard
            int dummy = 0;
            if (mSSBO[1] == 0) mSSBO[1] = makeSSBO(&dummy, sizeof(int));
        }
    }

    void reset() {
        glBindFramebuffer(GL_FRAMEBUFFER, mClearFBO);
        glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
    }

    void dispatch(int sampleCount, const Camera& cam, BRDFMode brdfMode, bool useNEE) {
        glUseProgram(mComputeProg);

        // Bind accumulation image
        glBindImageTexture(0, mAccumTex, 0, GL_FALSE, 0, GL_READ_WRITE, GL_RGBA32F);

        // Bind SSBOs (bindings 1–12)
        for (int i = 0; i < 12; i++)
            if (mSSBO[i]) glBindBufferBase(GL_SHADER_STORAGE_BUFFER, i + 1, mSSBO[i]);

        // Camera vectors
        Vec3 fwd = cam.forward;
        Vec3 right;
        Vec3 up0{0.0f, 1.0f, 0.0f};
        right.x = fwd.y*up0.z - fwd.z*up0.y;
        right.y = fwd.z*up0.x - fwd.x*up0.z;
        right.z = fwd.x*up0.y - fwd.y*up0.x;
        float rlen = std::sqrt(right.x*right.x+right.y*right.y+right.z*right.z);
        if (rlen < 0.001f) { right={1,0,0}; rlen=1.0f; }
        right.x/=rlen; right.y/=rlen; right.z/=rlen;
        Vec3 up; // cross(right, fwd)
        up.x=right.y*fwd.z - right.z*fwd.y;
        up.y=right.z*fwd.x - right.x*fwd.z;
        up.z=right.x*fwd.y - right.y*fwd.x;

        float scale  = std::tan(cam.fov * 0.5f * 3.14159265f / 180.0f);
        float aspect = (float)W / (float)H;

        auto loc = [&](const char* n){ return glGetUniformLocation(mComputeProg, n); };
        glUniform2i(loc("uSize"),      W, H);
        glUniform1i(loc("uSampleCount"), sampleCount);
        glUniform1i(loc("uNumSpheres"),  mNumSpheres);
        glUniform1i(loc("uNumBVHNodes"), mNumBVHNodes);
        glUniform1i(loc("uNumLights"),   mNumLights);
        glUniform1i(loc("uNumPlanes"),        mNumPlanes);
        glUniform1i(loc("uNumTori"),          mNumTori);
        glUniform1i(loc("uNumBoxes"),         mNumBoxes);
        glUniform1i(loc("uNumCapsules"),      mNumCapsules);
        glUniform1i(loc("uNumCylinders"),     mNumCylinders);
        glUniform1i(loc("uNumRoundedBoxes"),  mNumRoundedBoxes);
        glUniform1i(loc("uBRDFMode"),    brdfMode == BRDFMode::GGX ? 1 : 0);
        glUniform1i(loc("uUseNEE"),      useNEE ? 1 : 0);
        glUniform3f(loc("uCamPos"),      cam.pos.x,   cam.pos.y,   cam.pos.z);
        glUniform3f(loc("uCamFwd"),      fwd.x,       fwd.y,       fwd.z);
        glUniform3f(loc("uCamRight"),    right.x,     right.y,     right.z);
        glUniform3f(loc("uCamUp"),       up.x,        up.y,        up.z);
        glUniform1f(loc("uCamScale"),    scale);
        glUniform1f(loc("uCamAspect"),   aspect);

        // Bind scene textures to texture units 1..N (unit 0 reserved for accum in present)
        int numTex = (int)mTextures.size();
        glUniform1i(loc("uNumTextures"), numTex);
        for (int i = 0; i < numTex && i < 16; i++) {
            glActiveTexture(GL_TEXTURE1 + i);
            glBindTexture(GL_TEXTURE_2D, mTextures[i]);
            char name[32];
            std::snprintf(name, sizeof(name), "uTextures[%d]", i);
            glUniform1i(loc(name), 1 + i);
        }

        glDispatchCompute((W + 15) / 16, (H + 15) / 16, 1);
        glMemoryBarrier(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT | GL_TEXTURE_FETCH_BARRIER_BIT);
    }

    // Draw tone-mapped result to the current framebuffer (fullscreen triangle)
    void present(int sampleCount) {
        glUseProgram(mDisplayProg);
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, mAccumTex);
        glUniform1i(glGetUniformLocation(mDisplayProg, "uAccum"), 0);
        glUniform1i(glGetUniformLocation(mDisplayProg, "uSPP"),   sampleCount);
        glBindVertexArray(mVAO);
        glDrawArrays(GL_TRIANGLES, 0, 3);
        glBindVertexArray(0);
    }

    void destroy() {
        deleteSSBOs();
        deleteTextures();
        if (mAccumTex)    { glDeleteTextures(1,       &mAccumTex);    mAccumTex    = 0; }
        if (mClearFBO)    { glDeleteFramebuffers(1,   &mClearFBO);    mClearFBO    = 0; }
        if (mVAO)         { glDeleteVertexArrays(1,   &mVAO);         mVAO         = 0; }
        if (mComputeProg) { glDeleteProgram(mComputeProg);             mComputeProg = 0; }
        if (mDisplayProg) { glDeleteProgram(mDisplayProg);             mDisplayProg = 0; }
    }

private:
    GLuint mComputeProg = 0;
    GLuint mDisplayProg = 0;
    GLuint mAccumTex    = 0;
    GLuint mClearFBO    = 0;
    GLuint mVAO         = 0;
    GLuint mSSBO[12]    = {};
    std::vector<GLuint> mTextures;
    int    mNumSpheres      = 0;
    int    mNumBVHNodes     = 0;
    int    mNumLights       = 0;
    int    mNumPlanes       = 0;
    int    mNumTori         = 0;
    int    mNumBoxes        = 0;
    int    mNumCapsules     = 0;
    int    mNumCylinders    = 0;
    int    mNumRoundedBoxes = 0;

    static GLuint makeSSBO(const void* data, size_t bytes) {
        if (bytes == 0) {
            // Empty — upload a single zero int so binding is valid
            int dummy = 0;
            return makeSSBO(&dummy, sizeof(int));
        }
        GLuint buf;
        glGenBuffers(1, &buf);
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, buf);
        glBufferData(GL_SHADER_STORAGE_BUFFER, (GLsizeiptr)bytes,
                     data, GL_STATIC_DRAW);
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
        return buf;
    }

    void deleteSSBOs() {
        for (int i = 0; i < 12; i++)
            if (mSSBO[i]) { glDeleteBuffers(1, &mSSBO[i]); mSSBO[i] = 0; }
    }

    void deleteTextures() {
        if (!mTextures.empty()) {
            glDeleteTextures((GLsizei)mTextures.size(), mTextures.data());
            mTextures.clear();
        }
    }
};
