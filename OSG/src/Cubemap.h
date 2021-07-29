#pragma once
#include <osg/Group>
#include <osg/MatrixTransform>
#include <osgGA/KeySwitchMatrixManipulator>
#include <osgGA/FirstPersonManipulator>
#include <osgGA/TrackballManipulator>
#include <osg/ShapeDrawable>
#include <osg/TextureCubeMap>
#include <osg/Geode>
#include <osgViewer/Viewer>
#include <osgDB/ReadFile>
#include <osg/Depth>

#define M_PI   3.14159265358979323846264338327950288
#define degToRad(angleInDegrees) ((angleInDegrees) * M_PI / 180.0)
#define radToDeg(angleInRadians) ((angleInRadians) * 180.0 / M_PI)

class SkyboxTransform : public osg::Transform
{
public:
    // Get the transformation matrix which moves from local coords to world coords.
    virtual bool computeLocalToWorldMatrix(osg::Matrix& matrix,
        osg::NodeVisitor* nv) const
    {
        osgUtil::CullVisitor* cv = dynamic_cast<osgUtil::CullVisitor*>(nv);
        if (cv)
        {
            osg::Vec3 eyePointLocal = cv->getEyeLocal();
            osg::Matrixd preMatrix = osg::Matrix::rotate(degToRad(90),osg::Vec3(1,0,0)) * osg::Matrix::translate(eyePointLocal);
            matrix.preMult(preMatrix);
        }

        return true;
    }

    // Get the transformation matrix which moves from world coords to local coords.
    virtual bool computeWorldToLocalMatrix(osg::Matrix& matrix,
        osg::NodeVisitor* nv) const
    {
       /* osgUtil::CullVisitor* cv = dynamic_cast<osgUtil::CullVisitor*>(nv);
        if (cv)
        {
            osg::Vec3 eyePointLocal = cv->getEyeLocal();
            matrix.postMult(osg::Matrix::translate(eyePointLocal));
        }*/

        return true;
    }
};


osg::ref_ptr<osg::Node> createCubemap() {
    osg::ref_ptr<osg::TextureCubeMap> cubemap = new osg::TextureCubeMap;

    osg::ref_ptr<osg::Image> negx = osgDB::readImageFile("ExampleData/Cubemap_snow/negx.jpg");
    osg::ref_ptr<osg::Image> negy = osgDB::readImageFile("ExampleData/Cubemap_snow/negy.jpg");
    osg::ref_ptr<osg::Image> negz = osgDB::readImageFile("ExampleData/Cubemap_snow/negz.jpg");
    osg::ref_ptr<osg::Image> posx = osgDB::readImageFile("ExampleData/Cubemap_snow/posx.jpg");
    osg::ref_ptr<osg::Image> posy = osgDB::readImageFile("ExampleData/Cubemap_snow/posy.jpg");
    osg::ref_ptr<osg::Image> posz = osgDB::readImageFile("ExampleData/Cubemap_snow/posz.jpg");

    cubemap->setWrap(osg::Texture::WRAP_S, osg::Texture::CLAMP_TO_EDGE);
    cubemap->setWrap(osg::Texture::WRAP_T, osg::Texture::CLAMP_TO_EDGE);

    cubemap->setImage(osg::TextureCubeMap::POSITIVE_X, posx);
    cubemap->setImage(osg::TextureCubeMap::POSITIVE_Y, posy);
    cubemap->setImage(osg::TextureCubeMap::POSITIVE_Z, posz);
    cubemap->setImage(osg::TextureCubeMap::NEGATIVE_X, negx);
    cubemap->setImage(osg::TextureCubeMap::NEGATIVE_Y, negy);
    cubemap->setImage(osg::TextureCubeMap::NEGATIVE_Z, negz);

    

    // Create a drawable for the skybox
    osg::Geometry* drawable = new osg::Geometry;

    float radius = 100.f;
    // Create vertices for box
    osg::Vec3Array* verts = new osg::Vec3Array;
    verts->push_back(osg::Vec3(-radius, -radius, radius));
    verts->push_back(osg::Vec3(-radius, radius, radius));
    verts->push_back(osg::Vec3(radius, radius, radius));
    verts->push_back(osg::Vec3(radius, -radius, radius));
    verts->push_back(osg::Vec3(-radius, -radius, -radius));
    verts->push_back(osg::Vec3(-radius, radius, -radius));
    verts->push_back(osg::Vec3(radius, radius, -radius));
    verts->push_back(osg::Vec3(radius, -radius, -radius));
    drawable->setVertexArray(verts);

    // Create texture coordinates for cubemaps
    osg::Vec3Array* coords = new osg::Vec3Array;
    coords->push_back(osg::Vec3(-1, 1, -1));
    coords->push_back(osg::Vec3(-1, -1, -1));
    coords->push_back(osg::Vec3(1, -1, -1));
    coords->push_back(osg::Vec3(1, 1, -1));
    coords->push_back(osg::Vec3(-1, 1, 1));
    coords->push_back(osg::Vec3(-1, -1, 1));
    coords->push_back(osg::Vec3(1, -1, 1));
    coords->push_back(osg::Vec3(1, 1, 1));

    drawable->setTexCoordArray(0, coords);

    // Create an index array for the box
    osg::ref_ptr<osg::UIntArray> indices = new osg::UIntArray;

    // Front face
    indices->push_back(0);
    indices->push_back(1);
    indices->push_back(2);
    indices->push_back(3);

    // Back face
    indices->push_back(4);
    indices->push_back(5);
    indices->push_back(6);
    indices->push_back(7);

    // Right face
    indices->push_back(6);
    indices->push_back(7);
    indices->push_back(3);
    indices->push_back(2);

    // Left face
    indices->push_back(0);
    indices->push_back(1);
    indices->push_back(5);
    indices->push_back(4);

    // Top face
    indices->push_back(1);
    indices->push_back(2);
    indices->push_back(6);
    indices->push_back(5);

    // Bottom face
    indices->push_back(0);
    indices->push_back(3);
    indices->push_back(7);
    indices->push_back(4);

    drawable->addPrimitiveSet(
        new osg::DrawElementsUInt(GL_QUADS,
            indices->size(), &(indices->front())));

    // Create a geode for the skybox
    osg::Geode* geode = new osg::Geode;
    geode->setName("Skybox");
    // Disable culling
    geode->setCullingActive(false);
    // Add the skybox
    geode->addDrawable(drawable);

    geode->getOrCreateStateSet()->setTextureAttributeAndModes(0,cubemap);

    geode->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    geode->getOrCreateStateSet()->setMode(GL_CULL_FACE, osg::StateAttribute::OFF);
    // Make sure it is drawn last
    geode->getOrCreateStateSet()->setRenderBinDetails(1000, "RenderBin");

    // Clear the depth to the far plane.
    osg::Depth* depth = new osg::Depth(osg::Depth::LEQUAL, 1.0, 1.0);
    geode->getOrCreateStateSet()->setAttributeAndModes(depth, osg::StateAttribute::ON);

    osg::Transform* transform = new SkyboxTransform;
    transform->setCullingActive(false);
    transform->addChild(geode);
    transform->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
    transform->setName("SkyboxTransform");

    return transform;
}

