
#include <iostream>
#include <cmath>

#include "chrono/core/ChRealtimeStep.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono_cascade/ChBodyEasyCascade.h"
#include "chrono_cascade/ChCascadeDoc.h"
#include "chrono_cascade/ChCascadeShapeAsset.h"
#include "chrono_irrlicht/ChIrrApp.h"


using namespace std;

//branch test




// Use the namespace of Chrono
using namespace chrono;
using namespace chrono::irrlicht;

// Use the main namespaces of Irlicht
using namespace irr;
using namespace core;
using namespace scene;
using namespace video;
using namespace io;
using namespace gui;

// Use the namespace with OpenCascade stuff
using namespace cascade;


//static data for easy GUI manipulation
IGUIStaticText* text_motorspeed = 0;

//creating my event receiver class for interaction
class MyEventReceiver : public IEventReceiver {
public:
	MyEventReceiver(ChSystemNSC* system, IrrlichtDevice* device, std::shared_ptr<ChLinkMotorRotationSpeed> motor, std::shared_ptr<ChLinkMotorRotationSpeed> middleLeftMotor, shared_ptr<ChLinkMotorRotationSpeed> rearLeftMotor,
		shared_ptr<ChLinkMotorRotationSpeed> frontRightMotor, shared_ptr<ChLinkMotorRotationSpeed> middleRightMotor, shared_ptr<ChLinkMotorRotationSpeed> rearRightMotor) {
		//store pointer to physical system????

		msystem = system;
		mdevice = device;
		mmotor = motor;  //in code front left wheel 
		mmiddleLeftMotor = middleLeftMotor;
		mrearLeftMotor = rearLeftMotor; // in code rear left, in simulation rear right
		mfrontRightMotor = frontRightMotor;
		mmiddleRightMotor = middleRightMotor;
		mrearRightMotor = rearRightMotor;
	}
	
	bool OnEvent(const SEvent& event) {
		//check if user moved th sliders with mouse
		if (event.EventType == EET_GUI_EVENT) {
			s32 id = event.GUIEvent.Caller->getID();
			IGUIEnvironment* env = mdevice->getGUIEnvironment();

			switch (event.GUIEvent.EventType) {
			case EGET_SCROLL_BAR_CHANGED:
				if (id == 101) //id of motor speed slider???????
				{
					s32 pos = ((IGUIScrollBar*)event.GUIEvent.Caller)->getPos();
					double newspeed = 10 * (double)pos / 100.0;
					//set the speed into motor object
					if (auto mfun = std::dynamic_pointer_cast<ChFunction_Const>(mmotor->GetSpeedFunction()))
						mfun->Set_yconst(newspeed);
					if (auto mfun = dynamic_pointer_cast<ChFunction_Const>(mmiddleLeftMotor->GetSpeedFunction()))
						mfun->Set_yconst(newspeed);
					if (auto mfun = dynamic_pointer_cast<ChFunction_Const>(mrearLeftMotor->GetSpeedFunction()))
						mfun->Set_yconst(newspeed);
					if (auto mfun = dynamic_pointer_cast<ChFunction_Const>(mfrontRightMotor->GetSpeedFunction()))
						mfun->Set_yconst(newspeed);
					if (auto mfun = dynamic_pointer_cast<ChFunction_Const>(mmiddleRightMotor->GetSpeedFunction()))
						mfun->Set_yconst(newspeed);
					if (auto mfun = dynamic_pointer_cast<ChFunction_Const>(mrearRightMotor->GetSpeedFunction()))
						mfun->Set_yconst(newspeed);
					//show speed as ormatted text in interface screen
					char message[50];
					sprintf(message, "motor speed: %g [rad/s]", newspeed);
					text_motorspeed->setText(core::stringw(message).c_str());
				}
				break;
			default:
				break;
			}
		}
		return false;
	}
private:
	ChSystemNSC * msystem;
	IrrlichtDevice* mdevice;
	std::shared_ptr<ChLinkMotorRotationSpeed> mmotor;
	shared_ptr<ChLinkMotorRotationSpeed> mmiddleLeftMotor;
	shared_ptr<ChLinkMotorRotationSpeed> mrearLeftMotor;
	shared_ptr<ChLinkMotorRotationSpeed> mfrontRightMotor;
	shared_ptr<ChLinkMotorRotationSpeed> mmiddleRightMotor;
	shared_ptr<ChLinkMotorRotationSpeed> mrearRightMotor;
};



int main(int argc, char* argv[]) {

	SetChronoDataPath(CHRONO_DATA_DIR);

	// Create a ChronoENGINE physical system: all bodies and constraints
	// will be handled by this ChSystemNSC object.
	ChSystemNSC my_system;

	// Create the Irrlicht visualization (open the Irrlicht device,
	// bind a simple user interface, etc. etc.)
	ChIrrApp application(&my_system, L"rover simulation", core::dimension2d<u32>(800, 600), false, true);

	// Easy shortcuts to add logo, camera, lights and sky in Irrlicht scene:
	ChIrrWizard::add_typical_Logo(application.GetDevice());
	ChIrrWizard::add_typical_Sky(application.GetDevice());
	ChIrrWizard::add_typical_Lights(application.GetDevice(), core::vector3df(30, 100, 30), core::vector3df(30, -80, -30), 200, 130);
	ChIrrWizard::add_typical_Camera(application.GetDevice(), core::vector3df(0.2f, 0.2f, -0.3f));

	
	//works in roverSimulation also
	ChQuaternion<> rotation1;
	//rotation1.Q_from_AngAxis(CH_C_PI_2 , VECT_Z);  // 1: rotate 90� on X axis
	ChQuaternion<> rotation2;
	rotation2.Q_from_AngAxis(CH_C_PI, VECT_Y);            // 2: rotate 180� on vertical Y axis
	ChQuaternion<> tot_rotation = rotation2; //% rotation1;  // rotate on 1 then on 2, using quaternion product
	ChFrameMoving<> root_frame(ChVector<>(0, 0, 0), tot_rotation);


	

	//
	// Load a STEP file, containing a mechanism. The demo STEP file has been
	// created using a 3D CAD (in this case, SolidEdge v.18).
	//

	// Create the ChCascadeDoc, a container that loads the STEP model
	// and manages its subassembles
	ChCascadeDoc mydoc;

	// load the STEP model using this command:
	bool load_ok = mydoc.Load_STEP(GetChronoDataFile("/cascade/simplifiedAssembly.STEP").c_str());  // or specify abs.path: ("C:\\data\\cascade\\assembly.stp");


	mydoc.Dump(GetLog());		// print the contained shapes in console


	collision::ChCollisionModel::SetDefaultSuggestedEnvelope(0.002);
	collision::ChCollisionModel::SetDefaultSuggestedMargin(0.001);

	// In most CADs the Y axis is horizontal, but we want it vertical.
	// So define a root transformation for rotating all the imported objects.




	std::shared_ptr<ChBodyEasyCascade> mrigidBody1;		//connection axle
	std::shared_ptr<ChBodyEasyCascade> mrigidBody2;		//left rocker	
	std::shared_ptr<ChBodyEasyCascade> mrigidBody3;		//right rocker	
	std::shared_ptr<ChBodyEasyCascade> mrigidBody4;		//left bogey
	std::shared_ptr<ChBodyEasyCascade> mrigidBody5;		//right bogey

	std::shared_ptr<ChBody> bodyForCoordSys1;		//connection axle	
	std::shared_ptr<ChBody> bodyForCoordSys2;		//left rocker
	std::shared_ptr<ChBody> bodyForCoordSys3;		//right rocker
	std::shared_ptr<ChBody> bodyForCoordSys4;		//left bogey
	std::shared_ptr<ChBody> bodyForCoordSys5;		//right bogey
	std::shared_ptr<ChBody> bodyForCoordSys6;		//front left arm
	std::shared_ptr<ChBody> bodyForCoordSys7;		//rear left arm
	std::shared_ptr<ChBody> bodyForCoordSys8;		//front right arm
	std::shared_ptr<ChBody> bodyForCoordSys9;		//rear right arm
	std::shared_ptr<ChBody> bodyForCoordSys10;		//front left wheel
	std::shared_ptr<ChBody> bodyForCoordSys11;		//middle left wheel
	std::shared_ptr<ChBody> bodyForCoordSys12;		//rear left wheel
	std::shared_ptr<ChBody> bodyForCoordSys13;		//front right wheel
	std::shared_ptr<ChBody> bodyForCoordSys14;		//middle right wheel
	std::shared_ptr<ChBody> bodyForCoordSys15;		//rear right wheel
	std::shared_ptr<ChBody> bodyForCoordSys17;		//differential bar
	std::shared_ptr<ChBody> bodyForCoordSys18;		//diff bar connector left
	std::shared_ptr<ChBody> bodyForCoordSys19;		//diff bar connctor right


	if (load_ok) {

		//rover body
		TopoDS_Shape shape1;
		if (mydoc.GetNamedShape(shape1, "simplifiedAssembly/roverBody")) {

			std::shared_ptr<ChBodyEasyCascade> mbody1(new ChBodyEasyCascade(shape1, 1000, false, true));
			my_system.Add(mbody1);

			//mbody1->SetBodyFixed(true);
			mbody1->ConcatenatePreTransformation(root_frame);

			//mrigidBody1 = mbody1;
			bodyForCoordSys1 = mbody1;

		}
		else
			GetLog() << "Warning. connection axle not found \n";	


		//left rocker
		TopoDS_Shape shape2;
		if (mydoc.GetNamedShape(shape2, "simplifiedAssembly/LRockerMainFull")) {

			std::shared_ptr<ChBodyEasyCascade> mbody2(new ChBodyEasyCascade(shape2, 1000, false, true));
			my_system.Add(mbody2);

			//mbody2->SetBodyFixed(true);
			mbody2->ConcatenatePreTransformation(root_frame);

			//mrigidBody2 = mbody2;
			bodyForCoordSys2 = mbody2;

		}
		else
			GetLog() << "Warning. left main rocker not found \n";

		//right rocker
		TopoDS_Shape shape3;
		if (mydoc.GetNamedShape(shape3, "simplifiedAssembly/MirrorLRockerMainFull")) {

			std::shared_ptr<ChBodyEasyCascade> mbody3(new ChBodyEasyCascade(shape3, 1000, false, true));
			my_system.Add(mbody3);

			//mbody3->SetBodyFixed(true);
			mbody3->ConcatenatePreTransformation(root_frame);

			//mrigidBody3 = mbody3;
			bodyForCoordSys3 = mbody3;

		}
		else
			GetLog() << "Warning. right main rocker not found \n";

		//left bogey
		TopoDS_Shape shape4;
		if (mydoc.GetNamedShape(shape4, "simplifiedAssembly/LBogeyMain")) {

			std::shared_ptr<ChBodyEasyCascade> mbody4(new ChBodyEasyCascade(shape4, 1000, false, true));
			my_system.Add(mbody4);

			//mbody4->SetBodyFixed(true);
			mbody4->ConcatenatePreTransformation(root_frame);

			//mrigidBody3 = mbody3;
			bodyForCoordSys4 = mbody4;

		}
		else
			GetLog() << "Warning. left bogey not found \n";


		//right bogey
		TopoDS_Shape shape5;
		if (mydoc.GetNamedShape(shape5, "simplifiedAssembly/MirrorLBogeyMain")) {

			std::shared_ptr<ChBodyEasyCascade> mbody5(new ChBodyEasyCascade(shape5, 1000, false, true));
			my_system.Add(mbody5);

			//mbody5->SetBodyFixed(true);
			mbody5->ConcatenatePreTransformation(root_frame);

			//mrigidBody3 = mbody3;
			bodyForCoordSys5 = mbody5;

		}
		else
			GetLog() << "Warning. right bogey not found \n";

		//front left arm
		TopoDS_Shape shape6;
		if (mydoc.GetNamedShape(shape6, "simplifiedAssembly/LW_1-arm")) {

			std::shared_ptr<ChBodyEasyCascade> mbody6(new ChBodyEasyCascade(shape6, 1000, false, true));
			my_system.Add(mbody6);

			//mbody5->SetBodyFixed(true);
			mbody6->ConcatenatePreTransformation(root_frame);

			//mrigidBody3 = mbody3;
			bodyForCoordSys6 = mbody6;

		}
		else
			GetLog() << "Warning. left front arm not found \n";

		//rear left arm
		TopoDS_Shape shape7;
		if (mydoc.GetNamedShape(shape7, "simplifiedAssembly/LW_1-arm#2")) {

			std::shared_ptr<ChBodyEasyCascade> mbody7(new ChBodyEasyCascade(shape7, 1000, false, true));
			my_system.Add(mbody7);

			//mbody5->SetBodyFixed(true);
			mbody7->ConcatenatePreTransformation(root_frame);

			//mrigidBody3 = mbody3;
			bodyForCoordSys7 = mbody7;

		}
		else
			GetLog() << "Warning. left rear arm not found \n";


		//front right arm
		TopoDS_Shape shape8;
		if (mydoc.GetNamedShape(shape8, "simplifiedAssembly/LW_1-arm#3")) {

			std::shared_ptr<ChBodyEasyCascade> mbody8(new ChBodyEasyCascade(shape8, 1000, false, true));
			my_system.Add(mbody8);

			//mbody5->SetBodyFixed(true);
			mbody8->ConcatenatePreTransformation(root_frame);

			//mrigidBody3 = mbody3;
			bodyForCoordSys8 = mbody8;

		}
		else
			GetLog() << "Warning. front right arm not found \n";

		//rear right arm
		TopoDS_Shape shape9;
		if (mydoc.GetNamedShape(shape9, "simplifiedAssembly/LW_1-arm#4")) {

			std::shared_ptr<ChBodyEasyCascade> mbody9(new ChBodyEasyCascade(shape9, 1000, false, true));
			my_system.Add(mbody9);

			//mbody5->SetBodyFixed(true);
			mbody9->ConcatenatePreTransformation(root_frame);

			//mrigidBody3 = mbody3;
			bodyForCoordSys9 = mbody9;

		}
		else
			GetLog() << "Warning. right rear arm not found \n";

		//front left wheel
		TopoDS_Shape shape10;
		if (mydoc.GetNamedShape(shape10, "simplifiedAssembly/wheelMy")) {

			std::shared_ptr<ChBodyEasyCascade> mbody10(new ChBodyEasyCascade(shape10, 2710, true, true));
			my_system.Add(mbody10);

			//mbody10->SetBodyFixed(true);
			mbody10->ConcatenatePreTransformation(root_frame);

			//mrigidBody3 = mbody3;
			bodyForCoordSys10 = mbody10;

		}
		else
			GetLog() << "Warning. front left wheel not found \n";

		//middle left wheel
		TopoDS_Shape shape11;
		if (mydoc.GetNamedShape(shape11, "simplifiedAssembly/wheelMy#2")) {

			std::shared_ptr<ChBodyEasyCascade> mbody11(new ChBodyEasyCascade(shape11, 2710, true, true));
			my_system.Add(mbody11);

			//mbody10->SetBodyFixed(true);
			mbody11->ConcatenatePreTransformation(root_frame);

			//mrigidBody3 = mbody3;
			bodyForCoordSys11 = mbody11;

		}
		else
			GetLog() << "Warning. middle  left wheel not found \n";

		//rear left wheel
		TopoDS_Shape shape12;
		if (mydoc.GetNamedShape(shape12, "simplifiedAssembly/wheelMy#3")) {

			std::shared_ptr<ChBodyEasyCascade> mbody12(new ChBodyEasyCascade(shape12, 2710, true, true));
			my_system.Add(mbody12);

			//mbody10->SetBodyFixed(true);
			mbody12->ConcatenatePreTransformation(root_frame);

			//mrigidBody3 = mbody3;
			bodyForCoordSys12 = mbody12;

		}
		else
			GetLog() << "Warning. middle  left wheel not found \n";

		// front right wheel
		TopoDS_Shape shape13;
		if (mydoc.GetNamedShape(shape13, "simplifiedAssembly/wheelMy#4")) {

			std::shared_ptr<ChBodyEasyCascade> mbody13(new ChBodyEasyCascade(shape13, 2710, true, true));
			my_system.Add(mbody13);

			//mbody13->SetBodyFixed(true);
			mbody13->ConcatenatePreTransformation(root_frame);

			//mrigidBody3 = mbody3;
			bodyForCoordSys13 = mbody13;

		}
		else
			GetLog() << "Warning. front right wheel  not found \n";

		TopoDS_Shape shape14;
		if (mydoc.GetNamedShape(shape14, "simplifiedAssembly/wheelMy#5")) {

			std::shared_ptr<ChBodyEasyCascade> mbody14(new ChBodyEasyCascade(shape14, 2710, true, true));
			my_system.Add(mbody14);

			//mbody13->SetBodyFixed(true);
			mbody14->ConcatenatePreTransformation(root_frame);

			//mrigidBody3 = mbody3;
			bodyForCoordSys14 = mbody14;

		}
		else
			GetLog() << "Warning. middle right wheel not found \n";


		TopoDS_Shape shape15;
		if (mydoc.GetNamedShape(shape15, "simplifiedAssembly/wheelMy#6")) {

			std::shared_ptr<ChBodyEasyCascade> mbody15(new ChBodyEasyCascade(shape15, 2710, true, true));
			my_system.Add(mbody15);

			//mbody13->SetBodyFixed(true);
			mbody15->ConcatenatePreTransformation(root_frame);

			//mrigidBody3 = mbody3;
			bodyForCoordSys15 = mbody15;

		}
		else
			GetLog() << "Warning. virtual floor not found \n";


		//virtual floor
		TopoDS_Shape shape16;
		if (mydoc.GetNamedShape(shape16, "simplifiedAssembly/virtualFloor")) {

			std::shared_ptr<ChBodyEasyCascade> mbody16(new ChBodyEasyCascade(shape16, 1000000, true, true));
			my_system.Add(mbody16);

			mbody16->SetBodyFixed(true);
			mbody16->ConcatenatePreTransformation(root_frame);

			//mrigidBody3 = mbody3;
			//bodyForCoordSys15 = mbody15;

		}
		else
			GetLog() << "Warning. rear right wheel not found \n";
		

		//differential bar
		TopoDS_Shape shape17;
		if (mydoc.GetNamedShape(shape17, "simplifiedAssembly/differentialBar")) {

			std::shared_ptr<ChBodyEasyCascade> mbody17(new ChBodyEasyCascade(shape17, 1000, true, true));
			my_system.Add(mbody17);

			//mbody16->SetBodyFixed(true);
			mbody17->ConcatenatePreTransformation(root_frame);

			//mrigidBody3 = mbody3;
			bodyForCoordSys17 = mbody17;

		}
		else
			GetLog() << "Warning. rear right wheel not found \n";

		//left connector
		TopoDS_Shape shape18;
		if (mydoc.GetNamedShape(shape18, "simplifiedAssembly/sliderBase")) {

			std::shared_ptr<ChBodyEasyCascade> mbody18(new ChBodyEasyCascade(shape18, 1000, false, true));
			my_system.Add(mbody18);

			//mbody16->SetBodyFixed(true);
			mbody18->ConcatenatePreTransformation(root_frame);

			//mrigidBody3 = mbody3;
			bodyForCoordSys18 = mbody18;

		}
		else
			GetLog() << "Warning. rear right wheel not found \n";


		TopoDS_Shape shape19;
		if (mydoc.GetNamedShape(shape19, "simplifiedAssembly/sliderBase#2")) {

			std::shared_ptr<ChBodyEasyCascade> mbody19(new ChBodyEasyCascade(shape19, 1000, false, true));
			my_system.Add(mbody19);

			//mbody16->SetBodyFixed(true);
			mbody19->ConcatenatePreTransformation(root_frame);

			//mrigidBody3 = mbody3;
			bodyForCoordSys19 = mbody19;

		}
		else
			GetLog() << "Warning. rear right wheel not found \n";
	}
	else
		GetLog() << "Warning. Desired STEP file could not be opened/parsed \n";


	//retreiving marker shape coords front left wheel and arm
	

	//body and right rocker
	TopoDS_Shape shapeMarker;
	ChFrame<> referenceMarker;
	
	if (mydoc.GetNamedShape(shapeMarker, "simplifiedAssembly/markerCoordSys"))
		ChCascadeDoc::FromCascadeToChrono(shapeMarker.Location(), referenceMarker);
	else
	{
		GetLog() << "warning: desired marker not found in document - firstMarker \n";
	}
	referenceMarker %= root_frame;

	//body and left rocker
	TopoDS_Shape shapeMarker2;
	ChFrame<> referenceMarker2;

	if (mydoc.GetNamedShape(shapeMarker2, "simplifiedAssembly/markerCoordSys#2"))
		ChCascadeDoc::FromCascadeToChrono(shapeMarker2.Location(), referenceMarker2);
	else
	{
		GetLog() << "warning: desired marker not found in document - second marker \n";
	}
	referenceMarker2 %= root_frame;
	

	//left rocker and left bogey
	TopoDS_Shape shapeMarker3;
	ChFrame<> referenceMarker3;

	if (mydoc.GetNamedShape(shapeMarker3, "simplifiedAssembly/markerCoordSys#3"))
		ChCascadeDoc::FromCascadeToChrono(shapeMarker3.Location(), referenceMarker3);
	else
	{
		GetLog() << "warning: desired marker not found in document - third marker \n";
	}
	referenceMarker3 %= root_frame;

	//right rocker and right bogey
	TopoDS_Shape shapeMarker4;
	ChFrame<> referenceMarker4;

	if (mydoc.GetNamedShape(shapeMarker4, "simplifiedAssembly/markerCoordSys#4"))
		ChCascadeDoc::FromCascadeToChrono(shapeMarker4.Location(), referenceMarker4);
	else
	{
		GetLog() << "warning: desired marker not found in document - fourth marker \n";
	}
	referenceMarker4 %= root_frame;


	//front left arm and rocker 
	
	TopoDS_Shape shapeMarker5;
	ChFrame<> referenceMarker5;

	if (mydoc.GetNamedShape(shapeMarker5, "simplifiedAssembly/markerCoordSys#5"))
		ChCascadeDoc::FromCascadeToChrono(shapeMarker5.Location(), referenceMarker5);
	else
	{
		GetLog() << "warning: desired marker not found in document - fifth marker \n";
	}
	referenceMarker5 %= root_frame;
	

	//rear left arm and bogey
	TopoDS_Shape shapeMarker6;
	ChFrame<> referenceMarker6;

	if (mydoc.GetNamedShape(shapeMarker6, "simplifiedAssembly/markerCoordSys#6"))
		ChCascadeDoc::FromCascadeToChrono(shapeMarker6.Location(), referenceMarker6);
	else
	{
		GetLog() << "warning: desired marker not found in document - sixth marker \n";
	}
	referenceMarker6 %= root_frame;

	//front right arm and right rocker
	TopoDS_Shape shapeMarker7;
	ChFrame<> referenceMarker7;

	if (mydoc.GetNamedShape(shapeMarker7, "simplifiedAssembly/markerCoordSys#7"))
		ChCascadeDoc::FromCascadeToChrono(shapeMarker7.Location(), referenceMarker7);
	else
	{
		GetLog() << "warning: desired marker not found in document - seventh marker \n";
	}
	referenceMarker7 %= root_frame;

	//rear right arm and right bogey
	TopoDS_Shape shapeMarker8;
	ChFrame<> referenceMarker8;

	if (mydoc.GetNamedShape(shapeMarker8, "simplifiedAssembly/markerCoordSys#8"))
		ChCascadeDoc::FromCascadeToChrono(shapeMarker8.Location(), referenceMarker8);
	else
	{
		GetLog() << "warning: desired marker not found in document - eith marker \n";
	}
	referenceMarker8 %= root_frame;

	//front left arm and front left wheel
	TopoDS_Shape shapeMarker9;
	ChFrame<> referenceMarker9;

	if (mydoc.GetNamedShape(shapeMarker9, "simplifiedAssembly/markerCoordSys#9"))
		ChCascadeDoc::FromCascadeToChrono(shapeMarker9.Location(), referenceMarker9);
	else
	{
		GetLog() << "warning: desired marker not found in document - eith marker \n";
	}
	referenceMarker9 %= root_frame;

	//middle left wheel and left bogey
	TopoDS_Shape shapeMarker10;
	ChFrame<> referenceMarker10;

	if (mydoc.GetNamedShape(shapeMarker10, "simplifiedAssembly/markerCoordSys#10"))
		ChCascadeDoc::FromCascadeToChrono(shapeMarker10.Location(), referenceMarker10);
	else
	{
		GetLog() << "warning: desired marker not found in document - eith marker \n";
	}
	referenceMarker10 %= root_frame;

	//rear left wheel and arm
	TopoDS_Shape shapeMarker11;
	ChFrame<> referenceMarker11;

	if (mydoc.GetNamedShape(shapeMarker11, "simplifiedAssembly/markerCoordSys#11"))
		ChCascadeDoc::FromCascadeToChrono(shapeMarker11.Location(), referenceMarker11);
	else
	{
		GetLog() << "warning: desired marker not found in document - eleventh marker \n";
	}
	referenceMarker11 %= root_frame;

	//front right wheel and arm
	TopoDS_Shape shapeMarker12;
	ChFrame<> referenceMarker12;

	if (mydoc.GetNamedShape(shapeMarker12, "simplifiedAssembly/markerCoordSys#12"))
		ChCascadeDoc::FromCascadeToChrono(shapeMarker12.Location(), referenceMarker12);
	else
	{
		GetLog() << "warning: desired marker not found in document - twelfth marker \n";
	}
	referenceMarker12 %= root_frame;

	//middle right wheel and bogey right
	TopoDS_Shape shapeMarker13;
	ChFrame<> referenceMarker13;

	if (mydoc.GetNamedShape(shapeMarker13, "simplifiedAssembly/markerCoordSys#13"))
		ChCascadeDoc::FromCascadeToChrono(shapeMarker13.Location(), referenceMarker13);
	else
	{
		GetLog() << "warning: desired marker not found in document - twelfth marker \n";
	}
	referenceMarker13 %= root_frame;

	//rear right wheel and arm rear right
	TopoDS_Shape shapeMarker14;
	ChFrame<> referenceMarker14;

	if (mydoc.GetNamedShape(shapeMarker14, "simplifiedAssembly/markerCoordSys#14"))
		ChCascadeDoc::FromCascadeToChrono(shapeMarker14.Location(), referenceMarker14);
	else
	{
		GetLog() << "warning: desired marker not found in document - twelfth marker \n";
	}
	referenceMarker14 %= root_frame;


	//Differential bar and rover body
	TopoDS_Shape shapeMarker15;
	ChFrame<> referenceMarker15;

	if (mydoc.GetNamedShape(shapeMarker15, "simplifiedAssembly/markerCoordSys#15"))
		ChCascadeDoc::FromCascadeToChrono(shapeMarker15.Location(), referenceMarker15);
	else
	{
		GetLog() << "warning: desired marker not found in document - twelfth marker \n";
	}
	referenceMarker15 %= root_frame;


	//left rocker and connector spherical
	TopoDS_Shape shapeMarker16;
	ChFrame<> referenceMarker16;

	if (mydoc.GetNamedShape(shapeMarker16, "simplifiedAssembly/markerCoordSys#16"))
		ChCascadeDoc::FromCascadeToChrono(shapeMarker16.Location(), referenceMarker16);
	else
	{
		GetLog() << "warning: desired marker not found in document - twelfth marker \n";
	}
	referenceMarker16 %= root_frame;

	//differential bar and left connector spherical
	TopoDS_Shape shapeMarker17;
	ChFrame<> referenceMarker17;

	if (mydoc.GetNamedShape(shapeMarker17, "simplifiedAssembly/markerCoordSys#17"))
		ChCascadeDoc::FromCascadeToChrono(shapeMarker17.Location(), referenceMarker17);
	else
	{
		GetLog() << "warning: desired marker not found in document - twelfth marker \n";
	}
	referenceMarker17 %= root_frame;

	//differential bar and right connector spherical
	TopoDS_Shape shapeMarker18;
	ChFrame<> referenceMarker18;

	if (mydoc.GetNamedShape(shapeMarker18, "simplifiedAssembly/markerCoordSys#18"))
		ChCascadeDoc::FromCascadeToChrono(shapeMarker18.Location(), referenceMarker18);
	else
	{
		GetLog() << "warning: desired marker not found in document - twelfth marker \n";
	}
	referenceMarker18 %= root_frame;

	//right rocker and connector spherical
	TopoDS_Shape shapeMarker19;
	ChFrame<> referenceMarker19;

	if (mydoc.GetNamedShape(shapeMarker19, "simplifiedAssembly/markerCoordSys#19"))
		ChCascadeDoc::FromCascadeToChrono(shapeMarker19.Location(), referenceMarker19);
	else
	{
		GetLog() << "warning: desired marker not found in document - twelfth marker \n";
	}
	referenceMarker19 %= root_frame;





	// motor speed function
	auto motorSpeed2 = std::make_shared<ChFunction_Const>(-1*CH_C_PI); //speed	is 3.14 rad/sec


	//revolute link between connection axle and  left rocker

	auto revoluteJointLeft = make_shared<ChLinkRevolute>();
	revoluteJointLeft->Initialize(bodyForCoordSys1, bodyForCoordSys2, referenceMarker2);
	my_system.AddLink(revoluteJointLeft);


	//revolute link between connection axle and right rocker
	auto revoluteJointRight = make_shared<ChLinkRevolute>();
	revoluteJointRight->Initialize(bodyForCoordSys1, bodyForCoordSys3, referenceMarker);
	my_system.AddLink(revoluteJointRight);

	//revolute link between left rocker and left bogey
	auto revoluteJointLeftRB = make_shared<ChLinkRevolute>();
	revoluteJointLeftRB->Initialize(bodyForCoordSys2, bodyForCoordSys4, referenceMarker3);
	my_system.AddLink(revoluteJointLeftRB);

	//revolute link between right rocker and right bogey
	auto revoluteJointRightRB = make_shared<ChLinkRevolute>();
	revoluteJointRightRB->Initialize(bodyForCoordSys3, bodyForCoordSys5, referenceMarker4);
	my_system.AddLink(revoluteJointRightRB);

	//revolute link between front left arm and rocker - ///will be replaced with motor///
	//auto revoluteJointMotorAR = make_shared<ChLinkRevolute>();
	//revoluteJointMotorAR->Initialize(bodyForCoordSys2, bodyForCoordSys6, referenceMarker5);
	//my_system.AddLink(revoluteJointMotorAR);

	//fixed link front left
	auto fixedLinkFrontLeft = make_shared<ChLinkLockLock>();
	fixedLinkFrontLeft->Initialize(bodyForCoordSys2, bodyForCoordSys6, referenceMarker5.GetCoord());
	my_system.AddLink(fixedLinkFrontLeft);


	
	//revolute link between rear left arm and bogey -- ///to be replaced with motor///
	//auto revoluteJointMotorAB = make_shared<ChLinkRevolute>();
	//revoluteJointMotorAB->Initialize(bodyForCoordSys4, bodyForCoordSys7, referenceMarker6);
	//my_system.AddLink(revoluteJointMotorAB);

	//fixed link between rear left arm and bogey
	auto fixedLinkRearLeft = make_shared<ChLinkLockLock>();
	fixedLinkRearLeft->Initialize(bodyForCoordSys4, bodyForCoordSys7, referenceMarker6.GetCoord());
	my_system.AddLink(fixedLinkRearLeft);
	
	//revolute link between front right arm and rocker  -- //to be replaced with motor///
	//auto revoluteJointMotorARright = make_shared<ChLinkRevolute>();
	//revoluteJointMotorARright->Initialize(bodyForCoordSys3, bodyForCoordSys8, referenceMarker7);
	//my_system.AddLink(revoluteJointMotorARright);

	//fixed link between front right arm and rocker
	auto fixedLinkFrontRight = make_shared<ChLinkLockLock>();
	fixedLinkFrontRight->Initialize(bodyForCoordSys3, bodyForCoordSys8, referenceMarker7.GetCoord());
	my_system.AddLink(fixedLinkFrontRight);


	//revolute link between rear right arm and bogey  -- //to be replaced with motor///
	//auto revoluteJointMotorABright = make_shared<ChLinkRevolute>();
	//revoluteJointMotorABright->Initialize(bodyForCoordSys5, bodyForCoordSys9, referenceMarker8);
	//my_system.AddLink(revoluteJointMotorABright);

	//fixed link betwen rear rihgt arm and bogey
	auto fixedLinkRearRight = make_shared<ChLinkLockLock>();
	fixedLinkRearRight->Initialize(bodyForCoordSys5, bodyForCoordSys9, referenceMarker8.GetCoord());
	my_system.AddLink(fixedLinkRearRight);


	//revolute link between front left wheel and arm
	//auto revoluteMotorFrontLeft = make_shared<ChLinkRevolute>();
	//revoluteMotorFrontLeft->Initialize(bodyForCoordSys10, bodyForCoordSys6, referenceMarker9);
	//my_system.AddLink(revoluteMotorFrontLeft);


	//in code this is front left wheel 
	//MOTOR1   -- in simulation this is right wheel
	auto motor1 = make_shared<ChLinkMotorRotationSpeed>();
	motor1->Initialize(bodyForCoordSys10, bodyForCoordSys6, referenceMarker9);
	my_system.AddLink(motor1);
	motor1->SetMotorFunction(motorSpeed2);


	//revolute link between middle left wheel and left bogey	
	//auto revoluteMotorMiddleLeft = make_shared<ChLinkRevolute>();
	//revoluteMotorMiddleLeft->Initialize(bodyForCoordSys11, bodyForCoordSys4, referenceMarker10);
	//my_system.AddLink(revoluteMotorMiddleLeft);

	// MOTOR 2 - in code this is middle left wheel motor - in simulation is middle right wheel motor
	auto motor2 = make_shared<ChLinkMotorRotationSpeed>();
	motor2->Initialize(bodyForCoordSys11, bodyForCoordSys4, referenceMarker10);
	my_system.AddLink(motor2);
	motor2->SetMotorFunction(motorSpeed2);

	//revolute link between rear left wheel and rear left arm
	//auto revoluteMotorRearLeft = make_shared<ChLinkRevolute>();
	//revoluteMotorRearLeft->Initialize(bodyForCoordSys12, bodyForCoordSys7, referenceMarker11);
	//my_system.AddLink(revoluteMotorRearLeft);

	//MOTOR 3 - in code this is rear left wheel motor - in simulation this is right rear motor
	auto motor3 = std::make_shared<ChLinkMotorRotationSpeed>();
	motor3->Initialize(bodyForCoordSys12, bodyForCoordSys7, referenceMarker11);
	my_system.AddLink(motor3);
	motor3->SetMotorFunction(motorSpeed2);


	//revolute link between front right wheel and front right arm
	//auto revoluteMotorFrontRight = make_shared<ChLinkRevolute>();
	//revoluteMotorFrontRight->Initialize(bodyForCoordSys13, bodyForCoordSys8, referenceMarker12);
	//my_system.AddLink(revoluteMotorFrontRight);

	//MOTOR 4  - in code it is front right -  in simulation this is front left wheel 
	auto motor4 = make_shared<ChLinkMotorRotationSpeed>();
	motor4->Initialize(bodyForCoordSys13, bodyForCoordSys3, referenceMarker12);
	my_system.AddLink(motor4);
	motor4->SetMotorFunction(motorSpeed2);

	//revolute link between middle right wheel and right bogey
	//auto revoluteMotorMiddleRight = make_shared<ChLinkRevolute>();
	//revoluteMotorMiddleRight->Initialize(bodyForCoordSys14, bodyForCoordSys5, referenceMarker13);
	//my_system.AddLink(revoluteMotorMiddleRight);

	//MOTOR 5 - in code it is middle right - in simulation this is middle left wheel motor // NEEDS TO CHANGE MARKER ORIENTATION !!!!!
	auto motor5 = make_shared<ChLinkMotorRotationSpeed>();
	motor5->Initialize(bodyForCoordSys14, bodyForCoordSys5, referenceMarker13);
	my_system.AddLink(motor5);
	motor5->SetMotorFunction(motorSpeed2);

	//revolute link between rear right wheel and right arm
	//auto revoluteMotorRearRight = make_shared<ChLinkRevolute>();
	//revoluteMotorRearRight->Initialize(bodyForCoordSys15, bodyForCoordSys9, referenceMarker14);
	//my_system.AddLink(revoluteMotorRearRight);

	//MOTOR 6 - in code this is rear right wheel - in simulation this is rear left wheel ///// NEEDS TO REORIENT MARKER //////
	auto motor6 = make_shared<ChLinkMotorRotationSpeed>();
	motor6->Initialize(bodyForCoordSys15, bodyForCoordSys9, referenceMarker14);
	my_system.AddLink(motor6);
	motor6->SetMotorFunction(motorSpeed2);
	
	//revolute link between rover body and differential bar
	auto difBarBody = make_shared<ChLinkRevolute>();
	difBarBody->Initialize(bodyForCoordSys17, bodyForCoordSys1, referenceMarker15);
	my_system.AddLink(difBarBody);

	//spherical link between left rocker and left connector
	auto spheriOne = make_shared<ChLinkLockSpherical>();
	spheriOne->Initialize(bodyForCoordSys18, bodyForCoordSys2, referenceMarker16.GetCoord());
	my_system.AddLink(spheriOne);

	//spherical link between differential bar and left connector
	auto spheriTwo = make_shared<ChLinkLockSpherical>();
	spheriTwo->Initialize(bodyForCoordSys18, bodyForCoordSys17, referenceMarker17.GetCoord());
	my_system.AddLink(spheriTwo);

	//spherical link between differential bar and right connector
	auto spheriThree = make_shared<ChLinkLockSpherical>();
	spheriThree->Initialize(bodyForCoordSys17, bodyForCoordSys19, referenceMarker18.GetCoord());
	my_system.AddLink(spheriThree);

	//spherical link between differential bar and right rocker
	auto spheriFour = make_shared<ChLinkLockSpherical>();
	spheriFour->Initialize(bodyForCoordSys19, bodyForCoordSys3, referenceMarker19.GetCoord());
	my_system.AddLink(spheriFour);


	//create cube as floor
	shared_ptr<ChBodyEasyBox> mfloor(new ChBodyEasyBox(1, 0.1, 1, 1000, true));
	mfloor->SetPos(ChVector<>(1, -0 - 2, 0));
	mfloor->SetBodyFixed(true);
	my_system.AddBody(mfloor);

	

	//add a GUI text and GUI slider to control motor of mechanism via mouse
	text_motorspeed = application.GetIGUIEnvironment()->addStaticText(L"motor speed:", rect<s32>(300, 85, 400, 100), false);
	IGUIScrollBar* scrollbar = application.GetIGUIEnvironment()->addScrollBar(true, rect<s32>(300, 105, 450, 120), 0, 101);
	scrollbar->setMax(100);


	MyEventReceiver receiver(&my_system, application.GetDevice(), motor1, motor2, motor3, motor4, motor5, motor6);
	
	application.SetUserEventReceiver(&receiver);
		
	//comment out irrlicht chunck of code 
	

	// Use this function for adding a ChIrrNodeAsset to all items
	// Otherwise use application.AssetBind(myitem); on a per-item basis.
	application.AssetBindAll();

	// Use this function for 'converting' assets into Irrlicht meshes
	application.AssetUpdateAll();

	my_system.SetMaxItersSolverSpeed(100); // ??
	my_system.SetSolverType(ChSolver::Type::SOR_MULTITHREAD);		//solver SMC - ok

	auto forcesVect = motor4->Get_react_force();
	

	// THE SOFT-REAL-TIME CYCLE, SHOWING THE SIMULATION

	//shape marker 12 - front left wheel
	
	application.SetTimestep(0.005);
	application.SetTryRealtime(true);

	while (application.GetDevice()->run()) {
		// Irrlicht must prepare frame to draw
		application.BeginScene(true, true, video::SColor(255, 140, 161, 192));

		// .. draw solid 3D items (boxes, cylinders, shapes) belonging to Irrlicht scene, if any
		application.DrawAll();

		//ChIrrTools::drawAllCOGs(my_system, application.GetVideoDriver(), 0.1);
		GetLog() << "forces on front right wheel:  " << squareroot(pow(motor4->Get_react_force().x(),2)+pow(motor4->Get_react_force().y(),2)+pow(motor4->Get_react_force().z(),2)) << " \n";
		



		application.DoStep();

		application.EndScene();
	}

	

	return 0;
}