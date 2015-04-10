#ifndef WALKINGMANIPULATOR_H
#define WALKINGMANIPULATOR_H

#include <osgGA/StandardManipulator>

class WalkingManipulator : public osgGA::StandardManipulator
{
public:
    WalkingManipulator();

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

    void init( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& us );

    /** Set the position of the manipulator using a 4x4 matrix.*/
    void setByMatrix( const osg::Matrixd& matrix );

    /** Set the position of the manipulator using a 4x4 matrix.*/
    void setByInverseMatrix( const osg::Matrixd& matrix );

    /** Get the position of the manipulator as 4x4 matrix.*/
    osg::Matrixd getMatrix() const;

    /** Get the position of the manipulator as a inverse matrix of the manipulator,
    typically used as a model view matrix.*/
    osg::Matrixd getInverseMatrix() const;

    // doc in parent
    void setTransformation( const osg::Vec3d& eye, const osg::Quat& rotation );

    // doc in parent
    void getTransformation( osg::Vec3d& eye, osg::Quat& rotation ) const;

    // doc in parent
    void setTransformation( const osg::Vec3d& eye, const osg::Vec3d& center, const osg::Vec3d& up );

    // doc in parent
    void getTransformation( osg::Vec3d& eye, osg::Vec3d& center, osg::Vec3d& up ) const;

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
    bool handleFrame( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& us );

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

    void moveForward( const double distance );
    void moveRight( const double distance );

    /*void applyAnimationStep( const double currentProgress, const double prevProgress )
    {
        osgGA::FirstPersonManipulator::applyAnimationStep(currentProgress, prevProgress);
    }*/
    bool handleMouseMove( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& us );
    bool handleMousePush( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& us );
    bool handleMouseRelease( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& us );
    virtual bool handleKeyDown( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& us );
    virtual bool handleKeyUp( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& us );

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

#endif // WALKINGMANIPULATOR_H
