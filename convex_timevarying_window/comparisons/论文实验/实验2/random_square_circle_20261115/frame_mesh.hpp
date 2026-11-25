#pragma once
#include "geometry.hpp"
Json vec(const Vector&);
void quad(Json&m,const Vector&a,const Vector&b,const Vector&c,const Vector&d){m.push_back({vec(a),vec(b),vec(c)});m.push_back({vec(a),vec(c),vec(d)});}
Json frame_mesh(const Window&w){Json m=Json::array();constexpr double tube=.085;
 if(w.circle){constexpr int n=512,sides=12;auto at=[&](int k,int s)->Vector{double a=2*pi*k/n,b=2*pi*s/sides;return {(w.opening_radius+tube+tube*cos(b))*cos(a),(w.opening_radius+tube+tube*cos(b))*sin(a),tube*sin(b)};};for(int k=0;k<n;++k)for(int s=0;s<sides;++s)quad(m,at(k,s),at(k+1,s),at(k+1,s+1),at(k,s+1));}
 else for(int k=0;k<4;++k){Vector a(w.frame[k].x(),w.frame[k].y(),0),b(w.frame[(k+1)%4].x(),w.frame[(k+1)%4].y(),0),v=(b-a).normalized(),n(v.y(),-v.x(),0);auto at=[&](Vector p,int s)->Vector{double t=2*pi*s/12;return p+tube*cos(t)*n+tube*sin(t)*Vector::UnitZ();};for(int s=0;s<12;++s){quad(m,at(a,s),at(b,s),at(b,s+1),at(a,s+1));m.push_back({vec(a),vec(at(a,s+1)),vec(at(a,s))});m.push_back({vec(b),vec(at(b,s)),vec(at(b,s+1))});}}
 return m;}
