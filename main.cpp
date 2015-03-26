/* OpenSceneGraph example, osgterrain.
*
*  Permission is hereby granted, free of charge, to any person obtaining a copy
*  of this software and associated documentation files (the "Software"), to deal
*  in the Software without restriction, including without limitation the rights
*  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
*  copies of the Software, and to permit persons to whom the Software is
*  furnished to do so, subject to the following conditions:
*
*  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
*  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
*  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
*  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
*  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
*  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
*  THE SOFTWARE.
*/

#include <osg/ArgumentParser>
#include <osgDB/ReadFile>

#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>

#include <osgGA/TrackballManipulator>
#include <osgGA/FlightManipulator>
#include <osgGA/DriveManipulator>
#include <osgGA/KeySwitchMatrixManipulator>
#include <osgGA/StateSetManipulator>
#include <osgGA/AnimationPathManipulator>
#include <osgGA/TerrainManipulator>

#include <osgTerrain/Terrain>
#include <osgTerrain/TerrainTile>
#include <osgTerrain/GeometryTechnique>
#include <osgTerrain/Layer>

#include <iostream>

#include <osgDB/ReadFile>

template<class T>
class FindTopMostNodeOfTypeVisitor : public osg::NodeVisitor
{
public:
    FindTopMostNodeOfTypeVisitor():
        osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN),
        _foundNode(0)
    {}

    void apply(osg::Node& node)
    {
        T* result = dynamic_cast<T*>(&node);
        if (result)
        {
            _foundNode = result;
        }
        else
        {
            traverse(node);
        }
    }

    T* _foundNode;
};

template<class T>
T* findTopMostNodeOfType(osg::Node* node)
{
    if (!node) return 0;

    FindTopMostNodeOfTypeVisitor<T> fnotv;
    node->accept(fnotv);

    return fnotv._foundNode;
}

// class to handle events with a pick
class TerrainHandler : public osgGA::GUIEventHandler {
public:

    TerrainHandler(osgTerrain::Terrain* terrain):
        _terrain(terrain) {}

    bool handle(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter& aa)
    {
        switch(ea.getEventType())
        {
            case(osgGA::GUIEventAdapter::KEYDOWN):
            {
                if (ea.getKey()=='r')
                {
                    _terrain->setSampleRatio(_terrain->getSampleRatio()*0.5);
                    osg::notify(osg::NOTICE)<<"Sample ratio "<<_terrain->getSampleRatio()<<std::endl;
                    return true;
                }
                else if (ea.getKey()=='R')
                {
                    _terrain->setSampleRatio(_terrain->getSampleRatio()/0.5);
                    osg::notify(osg::NOTICE)<<"Sample ratio "<<_terrain->getSampleRatio()<<std::endl;
                    return true;
                }
                else if (ea.getKey()=='v')
                {
                    _terrain->setVerticalScale(_terrain->getVerticalScale()*1.25);
                    osg::notify(osg::NOTICE)<<"Vertical scale "<<_terrain->getVerticalScale()<<std::endl;
                    return true;
                }
                else if (ea.getKey()=='V')
                {
                    _terrain->setVerticalScale(_terrain->getVerticalScale()/1.25);
                    osg::notify(osg::NOTICE)<<"Vertical scale "<<_terrain->getVerticalScale()<<std::endl;
                    return true;
                }

                return false;
            }
            default:
                return false;
        }
    }

protected:

    ~TerrainHandler() {}

    osg::ref_ptr<osgTerrain::Terrain>  _terrain;
};

int main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc, argv);

    // construct the viewer.
    osgViewer::Viewer viewer(arguments);

    // set up the camera manipulators.
    {
        osg::ref_ptr<osgGA::KeySwitchMatrixManipulator> keyswitchManipulator = new osgGA::KeySwitchMatrixManipulator;

        keyswitchManipulator->addMatrixManipulator( '1', "Trackball", new osgGA::TrackballManipulator() );
        keyswitchManipulator->addMatrixManipulator( '2', "Flight", new osgGA::FlightManipulator() );
        keyswitchManipulator->addMatrixManipulator( '3', "Drive", new osgGA::DriveManipulator() );
        keyswitchManipulator->addMatrixManipulator( '4', "Terrain", new osgGA::TerrainManipulator() );

        std::string pathfile;
        char keyForAnimationPath = '5';
        while (arguments.read("-p",pathfile))
        {
            osgGA::AnimationPathManipulator* apm = new osgGA::AnimationPathManipulator(pathfile);
            if (apm || !apm->valid())
            {
                unsigned int num = keyswitchManipulator->getNumMatrixManipulators();
                keyswitchManipulator->addMatrixManipulator( keyForAnimationPath, "Path", apm );
                keyswitchManipulator->selectMatrixManipulator(num);
                ++keyForAnimationPath;
            }
        }

        viewer.setCameraManipulator( keyswitchManipulator.get() );
    }


    // add the state manipulator
    viewer.addEventHandler( new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()) );

    // add the stats handler
    viewer.addEventHandler(new osgViewer::StatsHandler);

    // add the record camera path handler
    viewer.addEventHandler(new osgViewer::RecordCameraPathHandler);

    // add the window size toggle handler
    viewer.addEventHandler(new osgViewer::WindowSizeHandler);

    // obtain the vertical scale
    float verticalScale = 1.0f;
    while(arguments.read("-v",verticalScale)) {}

    // obtain the sample ratio
    float sampleRatio = 1.0f;
    while(arguments.read("-r",sampleRatio)) {}

    osgTerrain::TerrainTile::BlendingPolicy blendingPolicy = osgTerrain::TerrainTile::INHERIT;
    std::string strBlendingPolicy;
    while(arguments.read("--blending-policy", strBlendingPolicy))
    {
        if (strBlendingPolicy == "INHERIT") blendingPolicy = osgTerrain::TerrainTile::INHERIT;
        else if (strBlendingPolicy == "DO_NOT_SET_BLENDING") blendingPolicy = osgTerrain::TerrainTile::DO_NOT_SET_BLENDING;
        else if (strBlendingPolicy == "ENABLE_BLENDING") blendingPolicy = osgTerrain::TerrainTile::ENABLE_BLENDING;
        else if (strBlendingPolicy == "ENABLE_BLENDING_WHEN_ALPHA_PRESENT") blendingPolicy = osgTerrain::TerrainTile::ENABLE_BLENDING_WHEN_ALPHA_PRESENT;
    }

    osg::ref_ptr<osg::Node> rootnode = new osg::Node;
    osg::Image* img = osgDB::readImageFile("terrain.png");

    osg::ref_ptr<osgTerrain::Locator> Locator1 = new osgTerrain::Locator;
    Locator1->setCoordinateSystemType( osgTerrain::Locator::PROJECTED );
    Locator1->setTransformAsExtents( 0.0, 0.0, 2560.0, 2560.0 );

    osg::ref_ptr<osg::HeightField> heightmap1 = new osg::HeightField;
    heightmap1 -> allocate(256,256);
    heightmap1 -> setXInterval(10.0f);
    heightmap1 -> setYInterval(10.0f);
    for(int z=0; z<256; z++)
    {
        for(int x=0; x<256; x++)
        {
            heightmap1->setHeight( x, z, (float)*img->data( x, z ) );
        }
    }

    osg::ref_ptr<osgTerrain::HeightFieldLayer> HeightFieldLayer1 = new osgTerrain::HeightFieldLayer( heightmap1.get() );
    HeightFieldLayer1->setLocator( Locator1.get() );

    osg::ref_ptr<osgTerrain::GeometryTechnique> GeometryTechnique1 = new osgTerrain::GeometryTechnique;
    osg::ref_ptr<osgTerrain::TerrainTile> TerrainTile1 = new osgTerrain::TerrainTile;
    TerrainTile1->setElevationLayer( HeightFieldLayer1.get() );
    TerrainTile1->setTerrainTechnique( GeometryTechnique1.get() );

    osg::ref_ptr<osgTerrain::Terrain> terrain = new osgTerrain::Terrain;
    terrain->setSampleRatio( 1.0f );

    //osg::ref_ptr<osg::Image> m_TextureImg = osgDB::readImageFile( _Texture );
    //osg::ref_ptr<osg::Texture2D> m_Texture = new osg::Texture2D;
    //m_Texture->setImage( m_TextureImg.get() );

    //osg::ref_ptr<osg::StateSet> m_TerrainStateSet = terrain -> getOrCreateStateSet();
    //m_TerrainStateSet->setTextureAttributeAndModes( 0, m_Texture, osg::StateAttribute::ShockedN );
    //m_TerrainStateSet->setMode( GL_LIGHTING, osg::StateAttribute::ShockedN );
    //m_TerrainStateSet->setAttribute( new osg::PolygonMode( osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::LINE ) );

    terrain -> addChild(TerrainTile1.get());

    rootnode = terrain.get();

    terrain->setSampleRatio(sampleRatio);
    terrain->setVerticalScale(verticalScale);
    terrain->setBlendingPolicy(blendingPolicy);

    // register our custom handler for adjust Terrain settings
    viewer.addEventHandler(new TerrainHandler(terrain.get()));

    // add a viewport to the viewer and attach the scene graph.
    viewer.setSceneData( rootnode.get() );

    // run the viewers main loop
    return viewer.run();

}
