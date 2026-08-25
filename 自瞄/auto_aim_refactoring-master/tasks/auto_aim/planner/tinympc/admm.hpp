#pragma once

#include "types.hpp"
//告诉编译器“有哪些函数/类型存在、它们的参数和返回值是什么”
#ifdef __cplusplus
//extern"C"：表示仅仅对C++有效，当用 C++ 编译器编译时，
//把括号里的函数声明变成 C 链接方式（避免 C++ 的 name mangling）。
//典型用途：让 C++ 写的库能被 C 调用，或者让 C/C++ 混编时符号名字一致。
extern "C" {
#endif

int solve(TinySolver *solver);

void update_primal(TinySolver *solver);
void backward_pass_grad(TinySolver *solver);
void forward_pass(TinySolver *solver);
void update_slack(TinySolver *solver);
void update_dual(TinySolver *solver);
void update_linear_cost(TinySolver *solver);
bool termination_condition(TinySolver *solver);

/**
 * Project a vector s onto the second order cone defined by mu
 * @param s, mu
 * @return projection onto cone if s is outside cone. Return s if s is inside cone.
*/
tinyVector project_soc(tinyVector s, float mu);

/**
 * Project a vector z onto a hyperplane defined by a^T z = b
 * Implements equation (21): ΠH(z) = z - (⟨z, a⟩ − b)/||a||² * a
 * @param z Vector to project
 * @param a Normal vector of the hyperplane
 * @param b Offset of the hyperplane
 * @return Projection of z onto the hyperplane
 */
tinyVector project_hyperplane(const tinyVector& z, const tinyVector& a, tinytype b);
#ifdef __cplusplus
}
#endif