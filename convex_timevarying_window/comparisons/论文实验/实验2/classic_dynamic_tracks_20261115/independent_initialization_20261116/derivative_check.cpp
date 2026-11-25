#include "native_minco_pvaj.hpp"
#include <filesystem>
int main(int argc,char** argv){try{
 Json m,s;std::ifstream(std::string(argv[1])+"/manifest.json")>>m;std::ifstream(argv[2])>>s;int n=m["n"],count=m["m"],vs=m["velocity_start"];auto original=m["initials"][0]["original"].get<std::vector<double>>();drolib::PVAJ head=drolib::PVAJ::Zero(),tail=head;
 for(int a=0;a<3;++a){head(a,0)=s["start"][a];tail(a,0)=s["goal"][a];}
 std::vector<double> x(n+12*count,0.);double at=0.;for(int k=0;k<n;++k){x[k]=std::exp(original[k]);at+=x[k];if(k<count){auto w=load_window(s["windows"][k]);Vector point=w.center+w.rotation*w.motion(at);for(int a=0;a<3;++a)x[n+3*k+a]=point[a];}}
 for(int k=0;k<9*count;++k)x[n+3*count+k]=original[vs+k];
 native_minco::Coefficients callback(head,tail,n,3);using casadi::MX;using casadi::DM;MX symbol=MX::sym("x",x.size());MX c=callback(std::vector<MX>{symbol})[0];MX scalar=MX::dot(c,c);MX gradient=MX::gradient(scalar,symbol);casadi::Function fn("audit",{symbol},{c,MX::jacobian(c,symbol),gradient,MX::jacobian(gradient,symbol)});
 auto actual=fn(std::vector<DM>{DM(x)});Json errors=Json::array();double maxJac=0.,maxHess=0.;
 for(int col:{0,n-1,n,n+3*count,n+6*count,n+9*count}){double h=1e-5*(1.+std::abs(x[col]));auto a=x,b=x;a[col]+=h;b[col]-=h;auto va=fn(std::vector<DM>{DM(a)}),vb=fn(std::vector<DM>{DM(b)});auto ca=va[0].get_elements(),cb=vb[0].get_elements(),ga=va[2].get_elements(),gb=vb[2].get_elements();double je=0.,he=0.;for(int row=0;row<ca.size();++row){double num=(ca[row]-cb[row])/(2*h),ana=static_cast<double>(actual[1](row,col));je=std::max(je,std::abs(num-ana)/(1.+std::abs(num)));}for(int row=0;row<ga.size();++row){double num=(ga[row]-gb[row])/(2*h),ana=static_cast<double>(actual[3](row,col));he=std::max(he,std::abs(num-ana)/(1.+std::abs(num)));}maxJac=std::max(maxJac,je);maxHess=std::max(maxHess,he);errors.push_back({{"column",col},{"jacobian_relative_error",je},{"hessian_relative_error",he}});}
 bool passed=maxJac<1e-4 && maxHess<1e-3;std::ofstream(argv[3])<<Json{{"passed",passed},{"max_jacobian_error",maxJac},{"max_hessian_error",maxHess},{"directions",errors}}.dump(2)<<'\n';std::cout<<passed<<' '<<maxJac<<' '<<maxHess<<'\n';return passed?0:1;
 }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
