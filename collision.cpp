#include "Utils.h"
#include "PickModelHandler.h"




//class MoveManipulator : public osgGA::GUIEventHandler
//{
//public:
//	MoveManipulator() : _co(NULL), _mt(NULL), lastColState(false){}
//    MoveManipulator( const MoveManipulator& mm, osg::CopyOp copyop ) : _co( mm._co ), _mt( mm._mt ) {}
//    ~MoveManipulator() {}
//#if( OSGWORKS_OSG_VERSION > 20800 )
//    META_Object(osgBulletExample,MoveManipulator);
//#endif
//	void detectCollision(bool& lastColState, btCollisionWorld* cw);
//
//    virtual bool handle( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa )
//    {
//        if( ( ea.getModKeyMask() & osgGA::GUIEventAdapter::MODKEY_CTRL ) == 0 )
//        {
//            return( false );
//        }
//        else if( ea.getEventType() == osgGA::GUIEventAdapter::PUSH )
//        {
//            _lastX = ea.getXnormalized();
//            _lastY = ea.getYnormalized();
//            return( true );
//        }
//        else if( ea.getEventType() == osgGA::GUIEventAdapter::DRAG )
//        {
//            double deltaX = ea.getXnormalized() - _lastX;
//            double deltaY = ea.getYnormalized() - _lastY;
//            _lastX = ea.getXnormalized();
//            _lastY = ea.getYnormalized();
//
//            deltaX *= 6.;
//            deltaY *= 6.;
//			osg::Matrix oriTrans = osgbCollision::asOsgMatrix(_co->getWorldTransform());
//
//            osg::Matrix trans = osgbCollision::asOsgMatrix( _co->getWorldTransform() );
//            trans = trans * osg::Matrix::translate( deltaX, 0., deltaY );
//            _mt->setMatrix( trans );
//            _co->setWorldTransform( osgbCollision::asBtTransform( trans ) );
//
//			collisionWorld->performDiscreteCollisionDetection();
//			detectCollision(lastColState, collisionWorld);
//			if (lastColState == true)
//			{
//				_mt->setMatrix(oriTrans);
//				_co->setWorldTransform(osgbCollision::asBtTransform(oriTrans));
//			}
//
//
//            return( true );
//        }
//        return( false );
//    }
//
//    void setCollisionObject( btCollisionObject* co ) { _co = co; }
//    void setMatrixTransform( osg::MatrixTransform* mt ) { _mt = mt; }
//	void setCollisionWorld(btCollisionWorld* btcw) { collisionWorld = btcw; }
//
//protected:
//    btCollisionObject* _co;
//    osg::MatrixTransform* _mt;
//	btCollisionWorld* collisionWorld;
//    double _lastX, _lastY;
//	bool lastColState;
//};
//
//void  MoveManipulator::detectCollision(bool& lastColState, btCollisionWorld* cw)
//{
//	unsigned int numManifolds = cw->getDispatcher()->getNumManifolds();
//	if ((numManifolds == 0) )
//	{
//		//osg::notify(osg::ALWAYS) << "No collision." << std::endl;
//		lastColState = false;
//	}
//	else {
//		for (unsigned int i = 0; i < numManifolds; i++)
//		{
//			btPersistentManifold* contactManifold = cw->getDispatcher()->getManifoldByIndexInternal(i);
//			unsigned int numContacts = contactManifold->getNumContacts();
//			for (unsigned int j = 0; j<numContacts; j++)
//			{
//				btManifoldPoint& pt = contactManifold->getContactPoint(j);
//				if ((pt.getDistance() <= 0.f) && (lastColState == false))
//				{
//					// grab these values for the contact normal arrows:
//					osg::Vec3 pos = osgbCollision::asOsgVec3(pt.getPositionWorldOnA()); // position of the collision on object A
//					osg::Vec3 normal = osgbCollision::asOsgVec3(pt.m_normalWorldOnB); // returns a unit vector
//					float pen = pt.getDistance(); //penetration depth
//
//					osg::Quat q;
//					q.makeRotate(osg::Vec3(0, 0, 1), normal);
//
//					osg::notify(osg::ALWAYS) << "Collision detected." << std::endl;
//
//					/*osg::notify(osg::ALWAYS) << "\tPosition: " << pos << std::endl;
//					osg::notify(osg::ALWAYS) << "\tNormal: " << normal << std::endl;
//					osg::notify(osg::ALWAYS) << "\tPenetration depth: " << pen << std::endl;*/
//					//osg::notify( osg::ALWAYS ) << q.w() <<","<< q.x() <<","<< q.y() <<","<< q.z() << std::endl;
//					lastColState = true;
//				}
//				else if ((pt.getDistance() > 0.f) && (lastColState == true))
//				{
//					//osg::notify(osg::ALWAYS) << "No collision." << std::endl;
//					lastColState = false;
//				}
//			}
//		}
//	}
//}


btCollisionWorld* initCollision()
{
    btDefaultCollisionConfiguration* collisionConfiguration = new btDefaultCollisionConfiguration();
    btCollisionDispatcher* dispatcher = new btCollisionDispatcher( collisionConfiguration );

    btVector3 worldAabbMin( -10000, -10000, -10000 );
    btVector3 worldAabbMax( 10000, 10000, 10000 );
    btBroadphaseInterface* inter = new btAxisSweep3( worldAabbMin, worldAabbMax, 1000 );

    btCollisionWorld* collisionWorld = new btCollisionWorld( dispatcher, inter, collisionConfiguration );

    return( collisionWorld );
}


osg::Group* createScene(btCollisionWorld* cw, PickModelHandler *picker, osg::ArgumentParser& arguments)
{
    osg::ref_ptr< osg::Group > root = new osg::Group;

    // Create a static box
   // osg::Geode* geode = new osg::Geode;
   // geode->addDrawable( osgwTools::makeBox( osg::Vec3( .5, .5, .5 ) ) );

	ref_ptr<Node> model1 = osgDB::readNodeFile("cow.osg");
	Matrix transMatrix1 = osg::Matrix::translate(-8., 0., 0.);
    ref_ptr<MatrixTransform> trans1 = new MatrixTransform(transMatrix1);
	trans1->addChild(model1.get());
	root->addChild(trans1.get());
	btCollisionObject* btBoxObject1 = new btCollisionObject;
	

	btBoxObject1->setCollisionShape(osgbCollision::btConvexHullCollisionShapeFromOSG(model1.get()));
	//btBoxObject1->setCollisionShape(osgbCollision::btBoxCollisionShapeFromOSG(model1.get()));
    //btBoxObject1->setCollisionFlags( btCollisionObject::CF_STATIC_OBJECT );
	btBoxObject1->setCollisionFlags(btCollisionObject::CF_KINEMATIC_OBJECT);
	btBoxObject1->setWorldTransform(osgbCollision::asBtTransform(transMatrix1));
    cw->addCollisionObject( btBoxObject1);
	
	picker->insertObjPair(trans1.get(), btBoxObject1);

    // Create a box we can drag around with the mouse
    //geode = new osg::Geode;
    //geode->addDrawable( osgwTools::makeBox( osg::Vec3( .5, .5, .5 ) ) );

	ref_ptr<Node> model2 = osgDB::readNodeFile("cow.osg");
    Matrix transMatrix2 = osg::Matrix::translate( 8., 0., 0. );
	ref_ptr<MatrixTransform> trans2 = new MatrixTransform(transMatrix2);
    trans2->addChild(model2.get());
    root->addChild(trans2.get());

	btCollisionObject* btBoxObject2 = new btCollisionObject;
	
	btBoxObject2->setCollisionShape(osgbCollision::btConvexHullCollisionShapeFromOSG(model2.get()));
    //btBoxObject2->setCollisionShape( osgbCollision::btBoxCollisionShapeFromOSG( model2.get()));
    btBoxObject2->setCollisionFlags( btCollisionObject::CF_KINEMATIC_OBJECT );
    btBoxObject2->setWorldTransform( osgbCollision::asBtTransform( transMatrix2 ) );
    cw->addCollisionObject( btBoxObject2 );

	picker->insertObjPair(trans2.get(), btBoxObject2);
	//btCollisionObjectArray btObjArray = cw->getCollisionObjectArray();
   // mm->setCollisionObject( btBoxObject );
    //mm->setMatrixTransform( trans2.get() );
	
    return( root.release() );
}



int main( int argc,
         char * argv[] )
{
    btCollisionWorld* collisionWorld = initCollision();

    osg::ArgumentParser arguments( &argc, argv );
   // MoveManipulator* mm = new MoveManipulator;
	//mm->setCollisionWorld(collisionWorld);
	ref_ptr<PickModelHandler> picker = new PickModelHandler;
	picker->setCollisionWorld(collisionWorld);

    osg::ref_ptr< osg::Group > root = createScene( collisionWorld, picker.get(), arguments );

	root->addChild(picker->getOrCreateSelectionBox());

    osgViewer::Viewer viewer;
    viewer.setUpViewInWindow( 10, 30, 800, 600 );
    viewer.setCameraManipulator( new osgGA::TrackballManipulator() );
    viewer.addEventHandler( picker.get() );
    viewer.setSceneData( root.get() );
	

    bool lastColState = false;
    while( !viewer.done() )
    {
       // collisionWorld->performDiscreteCollisionDetection();

       // detectCollision( lastColState, collisionWorld );

        viewer.frame();
    }

    return( 0 );
}



/** \page collision Using osgBullet For Collision Detection
osgBullet consists of two libraries, osgbCollision and osgbDynamics. This
library split allows your application to use Bullet for collision detection
with no dependency on libBulletDynamics, and render your results with OSG.
osgBullet contains an example program, \c collision, to demonstrate this usage.

\c collision renders two boxes. You can view them from any angle using the
OSG TrackballManipulator, but the example behaves more intuitively if you
use the default home position.

Move the box on the right by holding down the control key and dragging
with your left mouse button. If you drag the right box so that it is in
collision with the left box, the following message appears on the console:

\code
Collision detected.
        Position: 0.5 0.5 0.5
        Normal: -1 -0 -0
        Penetration depth: -5.96046e-008
\endcode

The \c Position, \c normal, and \c Penetration \c depth values are taken from the
Bullet collision information.

Drag the right box away from the left box and the following message appears
on the console:

\code
No collision.
\endcode

Using osgBullet, your application interfaces directly with
the Bullet API to determine if a collision has occurred, and if so, which
collision objects are in collision. The \c collision
example detects collisions by examining the manifold count in the Bullet collision
dispatcher, but Bullet provides other ways to detect collisions, as
discussed in the Bullet documentation and online forum.
*/
