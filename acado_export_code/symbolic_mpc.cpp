#include <acado_toolkit.hpp>
#include <acado_gnuplot.hpp>
#include <acado_code_generation.hpp>

using namespace std;

USING_NAMESPACE_ACADO

int main( )
{
   
        // INTRODUCE THE VARIABLES (acadoVariables.x):
        // -------------------------
        DifferentialState x; 
        DifferentialState y;
        DifferentialState q;

        Control v;
        Control w;
                
        double dt = 0.1;        // sampling time for discrete-time system

        // DEFINE A DIFFERENTIAL EQUATION of Kinematic Model:
        // -------------------------------
        DifferentialEquation f;
        // model equations
        f << dot(x) == v*cos(q);
        f << dot(y) == v*sin(q);
        f << dot(q) == w;

        //
        // Weighting matrices and reference functions (acadoVariables.y)
        //

        Function rf;
        Function rfN;

        rf  << x << y << q << v << w;
        rfN << x << y << q;

        const int N  = 30;
        const int Ni = 9;
        const double Ts = 0.1;

        // Provide defined weighting matrices:
        BMatrix W = eye<bool>(rf.getDim());
        BMatrix WN = eye<bool>(rfN.getDim());
        
        OCP ocp(0, N * Ts, N);

        ocp.subjectTo( f );
        // control constraints
        ocp.subjectTo( -3.0 <= v <= 3.0 );
        ocp.subjectTo( -0.785398 <= w <= 0.785398 );


        ocp.minimizeLSQ(W, rf); //cost
        ocp.minimizeLSQEndTerm(WN, rfN); //final cost

        // Export the code:
        OCPexport mpc( ocp );
        
        mpc.set(HESSIAN_APPROXIMATION, GAUSS_NEWTON);
        //mpc.set(DISCRETIZATION_TYPE, SINGLE_SHOOTING);        
        mpc.set(DISCRETIZATION_TYPE, MULTIPLE_SHOOTING);
        mpc.set(INTEGRATOR_TYPE, INT_RK45);
        // mpc.set(INTEGRATOR_TYPE, INT_IRK_RIIA3);
        mpc.set(NUM_INTEGRATOR_STEPS, N * Ni);
        mpc.set(SPARSE_QP_SOLUTION, FULL_CONDENSING);
        //	mpc.set(SPARSE_QP_SOLUTION, CONDENSING);
        mpc.set(QP_SOLVER, QP_QPOASES);
        //	mpc.set(QP_SOLVER, QP_FORCES);
        mpc.set(MAX_NUM_QP_ITERATIONS, 999);
        mpc.set(HOTSTART_QP, YES);        
        //	mpc.set(SPARSE_QP_SOLUTION, SPARSE_SOLVER);        
		mpc.set(LEVENBERG_MARQUARDT, 1.0E2);
        mpc.set(GENERATE_TEST_FILE, YES);
        mpc.set(GENERATE_MAKE_FILE, YES);
        mpc.set(GENERATE_MATLAB_INTERFACE, YES);
        //	mpc.set(USE_SINGLE_PRECISION, YES);
        mpc.set(CG_USE_VARIABLE_WEIGHTING_MATRIX, YES);
        mpc.set( CG_HARDCODE_CONSTRAINT_VALUES, NO);

        //mpc.set(CG_USE_OPENMP, YES);
        //NOTE: This is crucial for export of MHE!
	    //mpc.set(SPARSE_QP_SOLUTION, CONDENSING);
	    //mpc.set(FIX_INITIAL_STATE, YES);

        if (mpc.exportCode( "symbolic_mpc_export" ) != SUCCESSFUL_RETURN)
                exit( EXIT_FAILURE );

        mpc.printDimensionsQP( );

        return EXIT_SUCCESS;
}



