#include "mesh.hpp"
#include "lbfgsb.hpp"

void require(bool condition,const std::string& message) {
    if(!condition)throw std::runtime_error(message);
}
int main() {
    try {
        Curve curve;
        for(int i=0;i<8;++i)for(int j=0;j<3;++j)curve(i,j)=std::sin(3*i+j);
        const auto control=bernstein(curve);
        const auto enclosed=restrict_interval(control,.2,.8);
        const Vector low=enclosed.colwise().minCoeff().transpose(),high=enclosed.colwise().maxCoeff().transpose();
        for(int k=0;k<=1000;++k) {
            const Vector value=evaluate(curve,.2+.6*k/1000.);
            require((value.array()>=low.array()-1e-12).all() && (value.array()<=high.array()+1e-12).all(),"Bernstein interval enclosure");
        }
        Eigen::VectorXd polynomial(8);polynomial.setZero();polynomial[0]=-.2;polynomial[1]=1.;
        const auto solved=roots(polynomial);
        require(solved.size()==1 && std::abs(solved[0]-.2)<1e-12,"Polynomial trailing-zero roots");
        for(const auto& interval:std::vector<std::pair<double,double>>{{-10.,-1.},{-.1,.1},{1.,1.1},{10.,40.}}) {
            const auto range=sine_range(interval.first,interval.second);
            for(int k=0;k<=1000;++k) {
                const double value=std::sin(interval.first+(interval.second-interval.first)*k/1000.);
                require(value>=range.first-1e-12 && value<=range.second+1e-12,"Sine interval range");
            }
        }
        const std::array<Vector,3> triangle{Vector(0.,0.,0.),Vector(1.,0.,0.),Vector(0.,1.,0.)};
        require((closest_triangle(Vector(.2,.3,1.),triangle)-Vector(.2,.3,0.)).norm()<1e-12,"Triangle interior projection");
        require((closest_triangle(Vector(-1.,-1.,0.),triangle)-Vector::Zero()).norm()<1e-12,"Triangle vertex projection");
        LBFGSBOptions options;options.memory=10;options.ftol=1e-12;options.gtol=1e-8;options.max_iterations=2000;
        const auto rosenbrock=[](const std::vector<double>& x,std::vector<double>& gradient) {
            const double a=x[1]-x[0]*x[0],b=1.-x[0];
            gradient[0]=-400.*x[0]*a-2.*b;gradient[1]=200.*a;
            return 100.*a*a+b*b;
        };
        const auto optimum=minimize_lbfgsb(rosenbrock,{-1.2,1.},{-INFINITY,-INFINITY},{INFINITY,INFINITY},options);
        require(optimum.success && optimum.objective<1e-12,"Original L-BFGS-B unconstrained solve");
        const auto quadratic=[](const std::vector<double>& x,std::vector<double>& gradient) {
            gradient[0]=2.*(x[0]-2.);return (x[0]-2.)*(x[0]-2.);
        };
        const auto bounded=minimize_lbfgsb(quadratic,{0.},{-1.},{1.},options);
        require(bounded.success && std::abs(bounded.x[0]-1.)<1e-12,"Original L-BFGS-B physical input bound");
        std::cout<<"Native geometry, roots, triangle distance and optimizer checks passed\n";
    }catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
    return 0;
}
