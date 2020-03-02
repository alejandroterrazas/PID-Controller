using std::vector;

int max_n;
double total_cte = 0.0;
double error = 0.0;
double best_error = 10000.00;
double tol = 0.001;
int parameter_index = 0;
bool first = true;
bool done = false;
bool reset = true;
int skipframes;
bool initialize = true;
vector<double> error_vals;
int count = 0;
int interval;
int int_skip;
int offset_skip;
int offset;
int offset_count = 0;
vector<double> p;
vector<double> delta_p;
float steering_Kp_init;
float steering_Ki_init;
float steering_Kd_init;
float throttle_Kp_init;
float throttle_Ki_init;
float throttle_Kd_init;

float up_multiplier = 1.5;
float down_multiplier = .5;

//#ifndef M_PI
//const double M_PI = 3.14159265358979323846;
//#endif

///double = prev_cte = .5;
//double = int_cte = 0;

