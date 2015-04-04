#include "walkingmanipulator.h"

#include <osgGA/StandardManipulator>
#include <iostream>

WalkingManipulator::WalkingManipulator() : osgGA::StandardManipulator(),
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

void WalkingManipulator::init( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& us )
{
    _lastX = -1;//ea.getX();
    _lastY = -1;//ea.getY();
    home(ea, us);
    //std::cout << _lastX << ", " << _lastY << std::endl;
}

void WalkingManipulator::setByMatrix( const osg::Matrixd& matrix )
{
    // set variables
    _eye = matrix.getTrans();
    _rotation = matrix.getRotate();
    // fix current rotation
    //if( getVerticalAxisFixed() )
    //fixVerticalAxis( _eye, _rotation, true );
}

void WalkingManipulator::setByInverseMatrix( const osg::Matrixd& matrix )
{
    setByMatrix( osg::Matrixd::inverse( matrix ) );
}

osg::Matrixd WalkingManipulator::getMatrix() const
{
    return osg::Matrixd::rotate( _rotation ) * osg::Matrixd::translate( _eye );
}

osg::Matrixd WalkingManipulator::getInverseMatrix() const
{
    return osg::Matrixd::translate( -_eye ) * osg::Matrixd::rotate( _rotation.inverse() );
}

void WalkingManipulator::setTransformation( const osg::Vec3d& eye, const osg::Quat& rotation )
{
    // set variables
    _eye = eye;
    _rotation = rotation;
    // fix current rotation
    //if( getVerticalAxisFixed() )
    //fixVerticalAxis( _eye, _rotation, true );
}

void WalkingManipulator::getTransformation( osg::Vec3d& eye, osg::Quat& rotation ) const
{
    eye = _eye;
    rotation = _rotation;
}

void WalkingManipulator::setTransformation( const osg::Vec3d& eye, const osg::Vec3d& center, const osg::Vec3d& up )
{
    // set variables
    osg::Matrixd m( osg::Matrixd::lookAt( eye, center, up ) );
    _eye = eye;
    _rotation = m.getRotate().inverse();

    // fix current rotation
    //if( getVerticalAxisFixed() )
    //fixVerticalAxis( _eye, _rotation, true );
}

void WalkingManipulator::getTransformation( osg::Vec3d& eye, osg::Vec3d& center, osg::Vec3d& up ) const
{
    center = _eye + _rotation * osg::Vec3d( 0.,0.,-1. );
    eye = _eye;
    up = _rotation * osg::Vec3d( 0.,1.,0. );
}

bool WalkingManipulator::handleFrame( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& us )
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

void WalkingManipulator::moveForward( const double distance )
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

bool WalkingManipulator::handleMouseMove( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& us )
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

bool WalkingManipulator::handleMousePush( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& us )
{
    if (ea.getButton() == osgGA::GUIEventAdapter::RIGHT_MOUSE_BUTTON)
    {
        state = WALK_FORWARD;
        return true;
    }
    return false;
}

bool WalkingManipulator::handleMouseRelease( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& us )
{
    if (ea.getButton() == osgGA::GUIEventAdapter::RIGHT_MOUSE_BUTTON)
    {
        state = STAND;
    }
    return false;
}

bool WalkingManipulator::handleKeyDown( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& us )
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

bool WalkingManipulator::handleKeyUp( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& us )
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
