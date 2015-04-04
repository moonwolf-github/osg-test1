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

class WalkingManipulator : public osgGA::StandardManipulator
{
public:
    WalkingManipulator() : osgGA::StandardManipulator(),
        state(STAND), running(false)
        /*_thrown( false ),
        _allowThrow( true ),
        _mouseCenterX(0.0f), _mouseCenterY(0.0f),
        _delta_frame_time(0.01), _last_frame_time(0.0),
        _modelSize( 0. ),
        _verticalAxisFixed( true ),
        _flags( flags ),
        _relativeFlags( 0 )*/
    {

    }

    /*void home( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& us )
    {
        _homeEye.set(0.,-1.0,0.);
        _homeCenter.set(0.0,0.0,0.);
        _homeUp.set(0.0,0.0,1.0);
        const osg::Camera *camera = us.asView() ? us.asView()->getCamera() : NULL;
        computeHomePosition(camera, true);
        osgGA::CameraManipulator::home(ea, us);
        setTransformation( _homeEye, _homeCenter, _homeUp );
        _lastX = -1;//ea.getX();
        _lastY = -1;//ea.getY();
    }*/

    void init( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& us )
    {
        _lastX = -1;//ea.getX();
        _lastY = -1;//ea.getY();
        home(ea, us);
        //std::cout << _lastX << ", " << _lastY << std::endl;
    }

    /** Set the position of the manipulator using a 4x4 matrix.*/
    void setByMatrix( const osg::Matrixd& matrix )
    {
        // set variables
        _eye = matrix.getTrans();
        _rotation = matrix.getRotate();
        // fix current rotation
        //if( getVerticalAxisFixed() )
        //fixVerticalAxis( _eye, _rotation, true );
    }

    /** Set the position of the manipulator using a 4x4 matrix.*/
    void setByInverseMatrix( const osg::Matrixd& matrix )
    {
        setByMatrix( osg::Matrixd::inverse( matrix ) );
    }

    /** Get the position of the manipulator as 4x4 matrix.*/
    osg::Matrixd getMatrix() const
    {
        return osg::Matrixd::rotate( _rotation ) * osg::Matrixd::translate( _eye );
    }

    /** Get the position of the manipulator as a inverse matrix of the manipulator,
    typically used as a model view matrix.*/
    osg::Matrixd getInverseMatrix() const
    {
        return osg::Matrixd::translate( -_eye ) * osg::Matrixd::rotate( _rotation.inverse() );
    }

    // doc in parent
    void setTransformation( const osg::Vec3d& eye, const osg::Quat& rotation )
    {
        // set variables
        _eye = eye;
        _rotation = rotation;
        // fix current rotation
        //if( getVerticalAxisFixed() )
        //fixVerticalAxis( _eye, _rotation, true );
    }

    // doc in parent
    void getTransformation( osg::Vec3d& eye, osg::Quat& rotation ) const
    {
        eye = _eye;
        rotation = _rotation;
    }

    // doc in parent
    void setTransformation( const osg::Vec3d& eye, const osg::Vec3d& center, const osg::Vec3d& up )
    {
        // set variables
        osg::Matrixd m( osg::Matrixd::lookAt( eye, center, up ) );
        _eye = eye;
        _rotation = m.getRotate().inverse();

        // fix current rotation
        //if( getVerticalAxisFixed() )
        //fixVerticalAxis( _eye, _rotation, true );
    }

    // doc in parent
    void getTransformation( osg::Vec3d& eye, osg::Vec3d& center, osg::Vec3d& up ) const
    {
        center = _eye + _rotation * osg::Vec3d( 0.,0.,-1. );
        eye = _eye;
        up = _rotation * osg::Vec3d( 0.,1.,0. );
    }
    /*bool handle( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& us )
    {
        switch( ea.getEventType() )
        {
            case osgGA::GUIEventAdapter::FRAME:
                return handleFrame( ea, us );
            case osgGA::GUIEventAdapter::RESIZE:
                return handleResize( ea, us );
            default:
                break;
        }

        if( ea.getHandled() )
            return false;

        switch( ea.getEventType() )
        {
            case osgGA::GUIEventAdapter::MOVE:
                return handleMouseMove( ea, us );

            /case osgGA::GUIEventAdapter::DRAG:
                return handleMouseDrag( ea, us );/

            case osgGA::GUIEventAdapter::PUSH:
                return handleMousePush( ea, us );

            case osgGA::GUIEventAdapter::RELEASE:
                return handleMouseRelease( ea, us );

            case osgGA::GUIEventAdapter::KEYDOWN:
                return handleKeyDown( ea, us );

            case osgGA::GUIEventAdapter::KEYUP:
                return handleKeyUp( ea, us );

            /case GUIEventAdapter::SCROLL:
                if( _flags & PROCESS_MOUSE_WHEEL )
                    return handleMouseWheel( ea, us );
                else
                    return false;/

            default:
                return false;
        }
    }
    bool handleResize( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& us )
    {
        init( ea, us );
        us.requestRedraw();
        return true;
    }*/
protected:
    bool handleFrame( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& us )
    {
        switch (state)
        {
        case WALK_FORWARD:
            //std::cout << ea.getModKeyMask() << std::endl;
            if (running)
            {
                moveForward(80);
            }
            else
            {
                moveForward(20);
            }
            return true;
        case WALK_BACKWARD:
            if (running)
            {
                moveForward(-80);
            }
            else
            {
                moveForward(-20);
            }
            return true;
        default:
            return false;
        }
    }

    /*bool performMovementLeftMouseButton( const double eventTimeDelta, const double dx, const double dy )
    {
        return true;
    }*/

    /*void rotateYawPitch( osg::Quat& rotation, const double yaw, const double pitch, const osg::Vec3d& localUp)
    {
        // rotations
        osg::Quat rotateYaw( -yaw, rotation * osg::Vec3d( 0.,1.,0. ) );
        osg::Vec3d cameraRight( rotation * osg::Vec3d( 1.,0.,0. ) );
        osg::Quat rotatePitch( -pitch, cameraRight );
        osg::Quat newRotation = rotation * rotateYaw * rotatePitch;
        rotation = newRotation;
    }*/

    void moveForward( const double distance )
    {
        osg::Vec3d cameraForward;
        double angle;
        _rotation.getRotate(angle, cameraForward);
        cameraForward = _rotation * osg::Vec3d(0.,1.,0.);
        /*cameraForward.x() = 1.;
        cameraForward.y() = 0.;
        cameraForward.z() = 0.;*/
        std::cout << "angle: " << angle << " vec.x: " << cameraForward.x() << " vec.y: " << cameraForward.y() << " vec.z: " << cameraForward.z() << std::endl;

        osg::Quat t = osg::Quat(angle, cameraForward);
        std::cout << "x: " << t.x() << " y: " << t.y() << " z: " << t.z() << " w: " << t.w() << std::endl;
        /*t.x() = 0.707107;
        t.z() = 0;
        double mag = sqrt(t.y()*t.y() + t.w()*t.w());
        t.y() /= mag;
        t.w() /= mag;*/

        _eye += _rotation * osg::Vec3d( 0., 0., -distance );
    }

    /*void applyAnimationStep( const double currentProgress, const double prevProgress )
    {
        osgGA::FirstPersonManipulator::applyAnimationStep(currentProgress, prevProgress);
    }*/
    bool handleMouseMove( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& us )
    {
        if (_lastX < 0 && _lastY < 0)
        {
            _lastX = ea.getX();
            _lastY = ea.getY();
        }
        //return false;
        double dx = _lastX - ea.getX();
        double dy = _lastY - ea.getY();
        _lastX = ea.getX();
        _lastY = ea.getY();
        std::cout << dx << " " << dy << std::endl;
        //centerMousePointer(ea, us);
        // world up vector
        osg::CoordinateFrame coordinateFrame = getCoordinateFrame( _eye );
        osg::Vec3d localUp = getUpVector( coordinateFrame );
        rotateYawPitch( _rotation, -dx / 100., -dy / 100., localUp);
        osg::Quat t(_rotation);
        std::cout << "x: " << t.x() << " y: " << t.y() << " z: " << t.z() << " w: " << t.w() << std::endl;
        //us.requestWarpPointer(ea.getWindowWidth() / 2., ea.getWindowHeight() / 2.);
        return true;
    }
    bool handleMousePush( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& us )
    {
        if (ea.getButton() == osgGA::GUIEventAdapter::RIGHT_MOUSE_BUTTON)
        {
            state = WALK_FORWARD;
            return true;
        }
        return false;
    }
    bool handleMouseRelease( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& us )
    {
        if (ea.getButton() == osgGA::GUIEventAdapter::RIGHT_MOUSE_BUTTON)
        {
            state = STAND;
        }
        return false;
    }

    virtual bool handleKeyDown( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& us )
    {
        running = (ea.getModKeyMask() & osgGA::GUIEventAdapter::MODKEY_SHIFT) != 0;
        //std::cout << ea.getModKeyMask() << ", " << osgGA::GUIEventAdapter::MODKEY_SHIFT << ", " << (ea.getModKeyMask() & osgGA::GUIEventAdapter::MODKEY_SHIFT) << ", " << running << std::endl;
        if (ea.getKey() == osgGA::GUIEventAdapter::KEY_Up)
        {
            state = WALK_FORWARD;
            return true;
        }
        else if (ea.getKey() == osgGA::GUIEventAdapter::KEY_Down)
        {
            state = WALK_BACKWARD;
            return true;
        }
        else if (ea.getKey() == osgGA::GUIEventAdapter::KEY_Right)
        {
            state = WALK_BACKWARD;
            return true;
        }
        return false;
    }

    virtual bool handleKeyUp( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& us )
    {
        running = ea.getModKeyMask() & osgGA::GUIEventAdapter::MODKEY_SHIFT != 0;
        if (ea.getKey() == osgGA::GUIEventAdapter::KEY_Up)
        {
            state = STAND;
            return true;
        }
        else if (ea.getKey() == osgGA::GUIEventAdapter::KEY_Down)
        {
            state = STAND;
            return true;
        }
        return false;
    }
private:
    enum TState
    {
        WALK_FORWARD,
        WALK_BACKWARD,
        STRAFE_LEFT,
        STRAFE_RIGHT,
        STAND,
    };

    TState state;
    bool running;
    double _lastX;
    double _lastY;
    osg::Quat _rotation;
    osg::Vec3d _eye;
};

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

    osg::ref_ptr<osgTerrain::Terrain> _terrain;
};

int main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc, argv);

    // construct the viewer.
    osgViewer::Viewer viewer(arguments);

    // set up the camera manipulator.
    osg::ref_ptr<WalkingManipulator> manipulator = new WalkingManipulator;
    viewer.setCameraManipulator( manipulator.get() );

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
    osg::ref_ptr<osg::Image> img = osgDB::readImageFile("terrain.png");

    const int terrainXSize = img.get() -> s();
    const int terrainYSize = img.get() -> t();

    osg::ref_ptr<osgTerrain::Locator> Locator1 = new osgTerrain::Locator;
    Locator1->setCoordinateSystemType( osgTerrain::Locator::PROJECTED );
    Locator1->setTransformAsExtents( 0.0, 0.0, terrainXSize * 10.0, terrainYSize * 10.0 );

    osg::ref_ptr<osg::HeightField> heightmap1 = new osg::HeightField;
    heightmap1 -> allocate(terrainXSize, terrainYSize);
    heightmap1 -> setXInterval(10.0f);
    heightmap1 -> setYInterval(10.0f);
    for(int z=0; z<terrainYSize; z++)
    {
        for(int x=0; x<terrainXSize; x++)
        {
            heightmap1->setHeight( x, z, (float)*img.get()->data( x, z ) );
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

    osg::ref_ptr<osg::Image> m_TextureImg = osgDB::readImageFile( "tex.png" );
    osg::ref_ptr<osg::Texture2D> m_Texture = new osg::Texture2D;
    m_Texture->setImage( m_TextureImg.get() );

    osg::ref_ptr<osg::StateSet> m_TerrainStateSet = terrain -> getOrCreateStateSet();
    m_TerrainStateSet->setTextureAttributeAndModes( 0, m_Texture, osg::StateAttribute::ON );
    m_TerrainStateSet->setMode( GL_LIGHTING, osg::StateAttribute::ON );
    //m_TerrainStateSet->setAttribute( new osg::PolygonMode( osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::LINE ) );

    terrain -> addChild(TerrainTile1.get());

    osg::ref_ptr<osgTerrain::ImageLayer> imageLayer = new osgTerrain::ImageLayer;
    imageLayer->setImage(m_TextureImg.get());
    //imageLayer->setLocator(Locator1.get());
    TerrainTile1->setColorLayer(0, imageLayer.get());

    rootnode = terrain.get();

    //terrain->setSampleRatio(sampleRatio);
    terrain->setVerticalScale(verticalScale);
    terrain->setBlendingPolicy(blendingPolicy);

    // register our custom handler for adjust Terrain settings
    viewer.addEventHandler(new TerrainHandler(terrain.get()));

    // add a viewport to the viewer and attach the scene graph.
    viewer.setSceneData( rootnode.get() );

    // run the viewers main loop
    return viewer.run();

}
