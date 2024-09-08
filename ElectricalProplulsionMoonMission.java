package org.orekit.tutorials.deneme;


import org.hipparchus.geometry.euclidean.threed.Vector3D;
import org.hipparchus.ode.nonstiff.AdaptiveStepsizeIntegrator;
import org.hipparchus.ode.nonstiff.DormandPrince853Integrator;
import org.hipparchus.util.FastMath;
import org.orekit.bodies.CelestialBodyFactory;
import org.orekit.data.DataContext;
import org.orekit.data.DataProvidersManager;
import org.orekit.data.DirectoryCrawler;
import org.orekit.errors.OrekitException;
import org.orekit.forces.ForceModel;
import org.orekit.forces.gravity.HolmesFeatherstoneAttractionModel;
import org.orekit.forces.gravity.ThirdBodyAttraction;
import org.orekit.forces.gravity.potential.GravityFieldFactory;
import org.orekit.forces.gravity.potential.NormalizedSphericalHarmonicsProvider;
import org.orekit.forces.maneuvers.propulsion.BasicConstantThrustPropulsionModel;
import org.orekit.forces.maneuvers.propulsion.PropulsionModel;
import org.orekit.forces.maneuvers.trigger.ManeuverTriggers;
import org.orekit.frames.Frame;
import org.orekit.frames.FramesFactory;
import org.orekit.orbits.KeplerianOrbit;
import org.orekit.orbits.Orbit;
import org.orekit.orbits.OrbitType;
import org.orekit.orbits.PositionAngleType;
import org.orekit.propagation.SpacecraftState;
import org.orekit.propagation.numerical.NumericalPropagator;
import org.orekit.propagation.sampling.OrekitFixedStepHandler;
import org.orekit.time.AbsoluteDate;
import org.orekit.time.TimeScalesFactory;
import org.orekit.utils.Constants;
import org.orekit.utils.IERSConventions;

import java.io.File;
import java.util.Locale;

public class ElectricalProplulsionMoonMission {
    public static void main(String[] args) {
        try {
            final File home       = new File(System.getProperty("user.home"));
            final File orekitData = new File(home, "orekit-data");
            if (!orekitData.exists()) {
                System.err.format(Locale.US, "Failed to find %s folder%n",
                        orekitData.getAbsolutePath());
                System.err.format(Locale.US, "You need to download %s from %s, unzip it in %s and rename it 'orekit-data' for this tutorial to work%n",
                        "orekit-data-master.zip", "https://gitlab.orekit.org/orekit/orekit-data/-/archive/master/orekit-data-master.zip",
                        home.getAbsolutePath());
                System.exit(1);
            }
            final DataProvidersManager manager = DataContext.getDefault().getDataProvidersManager();
            manager.addProvider(new DirectoryCrawler(orekitData));

            // gravitation coefficient
            //final double mu =  3.986004415e+14;
            final double mu = Constants.EIGEN5C_EARTH_MU;

            // inertial frame
            final Frame inertialFrame = FramesFactory.getEME2000();

            // Initial date
            final AbsoluteDate initialDate = new AbsoluteDate(2000, 01, 01, 23, 30, 00.000,
                    TimeScalesFactory.getUTC());

            // Phase 1 end date
            final AbsoluteDate phaseOneEnd = initialDate.shiftedBy(81*24*60*60);

            // Initial orbit
            final double peri = 400000+Constants.EGM96_EARTH_EQUATORIAL_RADIUS; //perigee in meters
            final double apo = 125000000+Constants.EGM96_EARTH_EQUATORIAL_RADIUS; //apogee in meters
            final double a = (peri+apo)/2; // semi major axis in meters
            final double e = (apo-peri)/(apo+peri); // eccentricity
            final double i = FastMath.toRadians(7); // inclination
            final double omega = FastMath.toRadians(180); // perigee argument
            final double raan = FastMath.toRadians(0); // right ascention of ascending node
            final double w = 0; // true anomaly
            final Orbit initialOrbit = new KeplerianOrbit(a, e, i, omega, raan, w, PositionAngleType.MEAN,
                    inertialFrame, initialDate, mu);

            // Initial state definition
            final SpacecraftState initialState = new SpacecraftState(initialOrbit);
            Orbit orbit = initialOrbit;


            // Adaptive step integrator with a minimum step of 0.001 and a maximum step of 1000
            final double minStep = 0.001;
            final double maxstep = 1000.0;
            final double positionTolerance = 10.0;
            final OrbitType propagationType = OrbitType.KEPLERIAN;
            final double[][] tolerances =
                    NumericalPropagator.tolerances(positionTolerance, initialOrbit, propagationType);
            final AdaptiveStepsizeIntegrator integrator =
                    new DormandPrince853Integrator(minStep, maxstep, tolerances[0], tolerances[1]);

            // Propagator
            final NumericalPropagator propagator = new NumericalPropagator(integrator);
            propagator.setOrbitType(propagationType);

            // Force Model (reduced to perturbing gravity field)
            final NormalizedSphericalHarmonicsProvider provider =
                    GravityFieldFactory.getNormalizedProvider(10, 10);
            final ForceModel holmesFeatherstone =
                    new HolmesFeatherstoneAttractionModel(FramesFactory.getITRF(IERSConventions.IERS_2010,
                            true),
                            provider);

            propagator.addForceModel(new ThirdBodyAttraction(CelestialBodyFactory.getMoon()));

            // Add force model to the propagator
            propagator.addForceModel(holmesFeatherstone);

            // Set up initial state in the propagator
            propagator.setInitialState(initialState);

            // Set up a step handler
            propagator.getMultiplexer().add(60., new ElectricalProplulsionMoonMission.StepHandler());

            /*
            * BURAYI DOĞRULAMADAN DEVAM ETME!!!!!!!!!
            *
            *
            final Vector3D direction = new Vector3D(1,0,0);
            final AttitudeProvider attitudeOverride =
                    new FrameAlignedProvider(new Rotation(direction, Vector3D.PLUS_I), inertialFrame);
            */

            //velocity direction vector
            SpacecraftState spacecraftState = new SpacecraftState(orbit);
            Vector3D V = spacecraftState.getPVCoordinates().getVelocity().normalize();
            Vector3D thrustVector = V.negate();


            //thruster specs
            final double thrust = 0.07;
            final double isp    = 1500;
            final PropulsionModel propulsionModel =
                    new BasicConstantThrustPropulsionModel(thrust, isp,
                            thrustVector,"mainEngine");


            final ManeuverTriggers triggers = new Tr;

            if (initialDate.isBeforeOrEqualTo(phaseOneEnd)) {
                /*
                *
                * BURAYA LOOP GELECEK!!!
                * İLERLETME
                * TRIGGER
                * KONUM YAZDIRMA
                *
                *
                * */
            }









            // Extrapolate from the initial to the final date
            final SpacecraftState finalState = propagator.propagate(initialDate.shiftedBy(630.));
            final KeplerianOrbit o = (KeplerianOrbit) OrbitType.KEPLERIAN.convertType(finalState.getOrbit());
            System.out.format(Locale.US, "Final state:%n%s %12.3f %10.8f %10.6f %10.6f %10.6f %10.6f%n",
                    finalState.getDate(),
                    o.getA(), o.getE(),
                    FastMath.toDegrees(o.getI()),
                    FastMath.toDegrees(o.getPerigeeArgument()),
                    FastMath.toDegrees(o.getRightAscensionOfAscendingNode()),
                    FastMath.toDegrees(o.getTrueAnomaly()));

        } catch (OrekitException oe) {
            System.err.println(oe.getLocalizedMessage());
        }


    }
    private static class StepHandler implements OrekitFixedStepHandler {

        /** Simple constructor.
         */
        StepHandler() {
            //private constructor
        }

        /** {@inheritDoc} */
        @Override
        public void init(final SpacecraftState s0, final AbsoluteDate t, final double step) {
            System.out.println("          date                a           e" +
                    "           i         \u03c9          \u03a9" +
                    "          \u03bd");
        }

        /** {@inheritDoc} */
        @Override
        public void handleStep(final SpacecraftState currentState) {
            final KeplerianOrbit o = (KeplerianOrbit) OrbitType.KEPLERIAN.convertType(currentState.getOrbit());
            System.out.format(Locale.US, "%s %12.3f %10.8f %10.6f %10.6f %10.6f %10.6f%n",
                    currentState.getDate(),
                    o.getA(), o.getE(),
                    FastMath.toDegrees(o.getI()),
                    FastMath.toDegrees(o.getPerigeeArgument()),
                    FastMath.toDegrees(o.getRightAscensionOfAscendingNode()),
                    FastMath.toDegrees(o.getTrueAnomaly()));
        }

        /** {@inheritDoc} */
        @Override
        public void finish(final SpacecraftState finalState) {
            System.out.println("this was the last step ");
            System.out.println();
        }
    }
}
