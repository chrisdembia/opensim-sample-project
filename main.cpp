#include <OpenSim/OpenSim.h>
using namespace SimTK;
using namespace OpenSim; using OpenSim::Body;
int main() {
    Model model; model.setUseVisualizer(true);
    // Two links, with mass of 1 kg, center of mass at the
    // origin of the body's frame, and moments/products of inertia of
    // zero.
    OpenSim::Body* link1 = new OpenSim::Body("humerus", 1, Vec3(0),
            Inertia(0));
    OpenSim::Body* link2 = new OpenSim::Body("radius", 1, Vec3(0),
            Inertia(0));

    // Joints that connect the bodies together.
    PinJoint* joint1 = new PinJoint("shoulder",
            // Parent body, location in parent, orientation in parent.
            model.getGroundBody(), Vec3(0), Vec3(0),
            // Child body, location in child, orientation in child.
            *link1, Vec3(0, 1, 0), Vec3(0));
    PinJoint* joint2 = new PinJoint("elbow",
            *link1, Vec3(0), Vec3(0), *link2,
            Vec3(0, 1, 0), Vec3(0));

    // Add an actuator that crosses the elbow, and a joint stop.
    Millard2012EquilibriumMuscle* muscle = new
        Millard2012EquilibriumMuscle("biceps", 200, 0.6, 0.55, 0);
    muscle->addNewPathPoint("point1", *link1, Vec3(0, 0.8, 0));
    muscle->addNewPathPoint("point2", *link2, Vec3(0, 0.7, 0));

    // A controller that specifies the excitation of the biceps muscle.
    PrescribedController* brain = new PrescribedController();
    brain->addActuator(*muscle);
    // Muscle excitation is 0.3 for
    // the first 0.5 seconds, and
    // 1.0 thereafter.
    brain->prescribeControlForActuator("biceps",
            new StepFunction(0.5, 3, 0.3, 1));

    // Add bodies and joints to the model.
    model.addBody(link1);
    model.addBody(link2);
    model.addJoint(joint1);
    model.addJoint(joint2);
    model.addForce(muscle);
    model.addController(brain);

    // Configure the model.
    State& state = model.initSystem();
    // Fix shoulder joint, flex elbow joint.
    model.updCoordinateSet()[0].setLocked(state, true);
    model.updCoordinateSet()[1].setValue(state, 0.5 * Pi);
    model.equilibrateMuscles(state);

    // Add display geometry.
    model.updMatterSubsystem().setShowDefaultGeometry(true);
    Visualizer& viz = model.updVisualizer().updSimbodyVisualizer();
    viz.setBackgroundColor(Vec3(1, 1, 1));
    // Ellipsoids: 0.5 m radius along y axis, centered 0.5 m up along y axis.
    DecorativeEllipsoid geom(Vec3(0.1, 0.5, 0.1));
    Vec3 center(0, 0.5, 0);
    viz.addDecoration(link1->getIndex(), Transform(center), geom);
    viz.addDecoration(link2->getIndex(), Transform(center), geom);

    // Simulate.
    RungeKuttaMersonIntegrator integrator(model.getSystem());
    Manager manager(model, integrator);
    manager.setInitialTime(0);
    manager.setFinalTime(10.0);
    manager.integrate(state);
};
