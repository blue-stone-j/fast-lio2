#ifndef EXPAND_MACROS_H
#define EXPAND_MACROS_H

#include <IKFoM_toolkit/esekfom/esekfom.hpp>

typedef MTK::vect<3, double> vect3;
typedef MTK::SO3<double> SO3;
typedef MTK::S2<double, 98090, 10000, 1> S2; // S2 流形
typedef MTK::vect<1, double> vect1;
typedef MTK::vect<2, double> vect2;

/********************state_ikfom***********************/
// 定义的ieskf状态空间; R_L_I: rotation between lidar and inertia
MTK_BUILD_MANIFOLD(state_ikfom,
                   ((vect3, pos))((SO3, rot))((SO3, offset_R_L_I)) // rotation from lidar to imu
                   ((vect3, offset_T_L_I)) // translation from lidar to imu
                   ((vect3, vel))((vect3, bg))((vect3, ba)) // S2流形,grav为负值
                   ((S2, grav)));
struct state_ikfom
{
  typedef state_ikfom self;
  std::vector<std::pair<int, int>> S2_state;
  std::vector<std::pair<int, int>> SO3_state;
  std::vector<std::pair<std::pair<int, int>, int>> vect_state;
  // MTK_ENTRIES_OUTPUT_I ( 8, (vect3, pos) , ((SO3, rot)) ((SO3, offset_R_L_I)) ((vect3, offset_T_L_I)) ((vect3, vel)) ((vect3, bg)) ((vect3, ba)) ((S2, grav)) (~), 0, 0, S2_state, SO3_state )
  MTK::SubManifold<vect3, 0, 0> pos;

  // MTK_ENTRIES_OUTPUT_I ( 7, (SO3, rot) ,  ((SO3, offset_R_L_I)) ((vect3, offset_T_L_I)) ((vect3, vel)) ((vect3, bg)) ((vect3, ba)) ((S2, grav)) (~), 0 + BOOST_PP_TUPLE_ELEM_2_0 (vect3, pos)::DOF, 0 + BOOST_PP_TUPLE_ELEM_2_0 (vect3, pos)::DIM, S2_state, SO3_state) MTK_ENTRIES_OUTPUT_I ( 6, (SO3, offset_R_L_I) ,  ((vect3, offset_T_L_I)) ((vect3, vel)) ((vect3, bg)) ((vect3, ba)) ((S2, grav)) (~), 0 + vect3::DOF + BOOST_PP_TUPLE_ELEM_2_0 (SO3, rot)::DOF, 0 + vect3::DIM + BOOST_PP_TUPLE_ELEM_2_0 (SO3, rot)::DIM, S2_state, SO3_state)
  MTK::SubManifold<SO3, 0 + vect3::DOF, 0 + vect3::DIM> rot;
  MTK::SubManifold<SO3, 0 + vect3::DOF + SO3::DOF, 0 + vect3::DIM + SO3::DIM> offset_R_L_I;

  // MTK_ENTRIES_OUTPUT_I ( 5, (vect3, offset_T_L_I) ,  ((vect3, vel)) ((vect3, bg)) ((vect3, ba)) ((S2, grav)) (~), 0 + vect3::DOF + SO3::DOF + BOOST_PP_TUPLE_ELEM_2_0 (SO3, offset_R_L_I)::DOF, 0 + vect3::DIM + SO3::DIM + BOOST_PP_TUPLE_ELEM_2_0 (SO3, offset_R_L_I)::DIM, S2_state, SO3_state) MTK_ENTRIES_OUTPUT_I ( 4, (vect3, vel) ,  ((vect3, bg)) ((vect3, ba)) ((S2, grav)) (~), 0 + vect3::DOF + SO3::DOF + SO3::DOF + BOOST_PP_TUPLE_ELEM_2_0 (vect3, offset_T_L_I)::DOF, 0 + vect3::DIM + SO3::DIM + SO3::DIM + BOOST_PP_TUPLE_ELEM_2_0 (vect3, offset_T_L_I)::DIM, S2_state, SO3_state)
  MTK::SubManifold<vect3, 0 + vect3::DOF + SO3::DOF + SO3::DOF, 0 + vect3::DIM + SO3::DIM + SO3::DIM> offset_T_L_I;
  MTK::SubManifold<vect3, 0 + vect3::DOF + SO3::DOF + SO3::DOF + vect3::DOF, 0 + vect3::DIM + SO3::DIM + SO3::DIM + vect3::DIM> vel;

  // MTK_ENTRIES_OUTPUT_I ( 3, (vect3, bg) ,  ((vect3, ba)) ((S2, grav)) (~), 0 + vect3::DOF + SO3::DOF + SO3::DOF + vect3::DOF + BOOST_PP_TUPLE_ELEM_2_0 (vect3, vel)::DOF, 0 + vect3::DIM + SO3::DIM + SO3::DIM + vect3::DIM + BOOST_PP_TUPLE_ELEM_2_0 (vect3, vel)::DIM, S2_state, SO3_state) MTK_ENTRIES_OUTPUT_I ( 2, (vect3, ba) ,  ((S2, grav)) (~), 0 + vect3::DOF + SO3::DOF + SO3::DOF + vect3::DOF + vect3::DOF + BOOST_PP_TUPLE_ELEM_2_0 (vect3, bg)::DOF, 0 + vect3::DIM + SO3::DIM + SO3::DIM + vect3::DIM + vect3::DIM + BOOST_PP_TUPLE_ELEM_2_0 (vect3, bg)::DIM, S2_state, SO3_state)
  MTK::SubManifold<vect3, 0 + vect3::DOF + SO3::DOF + SO3::DOF + vect3::DOF + vect3::DOF, 0 + vect3::DIM + SO3::DIM + SO3::DIM + vect3::DIM + vect3::DIM> bg;
  MTK::SubManifold<vect3, 0 + vect3::DOF + SO3::DOF + SO3::DOF + vect3::DOF + vect3::DOF + vect3::DOF, 0 + vect3::DIM + SO3::DIM + SO3::DIM + vect3::DIM + vect3::DIM + vect3::DIM> ba;

  // MTK_ENTRIES_OUTPUT_I ( 1, (S2, grav) ,  (~), 0 + vect3::DOF + SO3::DOF + SO3::DOF + vect3::DOF + vect3::DOF + vect3::DOF + BOOST_PP_TUPLE_ELEM_2_0 (vect3, ba)::DOF, 0 + vect3::DIM + SO3::DIM + SO3::DIM + vect3::DIM + vect3::DIM + vect3::DIM + BOOST_PP_TUPLE_ELEM_2_0 (vect3, ba)::DIM, S2_state, SO3_state)
  MTK::SubManifold<S2, 0 + vect3::DOF + SO3::DOF + SO3::DOF + vect3::DOF + vect3::DOF + vect3::DOF + vect3::DOF, 0 + vect3::DIM + SO3::DIM + SO3::DIM + vect3::DIM + vect3::DIM + vect3::DIM + vect3::DIM> grav;
  enum
  {
    DOF = S2::DOF + 0 + vect3::DOF + SO3::DOF + SO3::DOF + vect3::DOF + vect3::DOF + vect3::DOF + vect3::DOF
  };
  enum
  {
    DIM = S2::DIM + 0 + vect3::DIM + SO3::DIM + SO3::DIM + vect3::DIM + vect3::DIM + vect3::DIM + vect3::DIM
  };
  typedef S2::scalar scalar;

  // state_ikfom ( BOOST_PP_SEQ_ENUM_8  (const vect3& pos = vect3()) (const SO3& rot = SO3()) (const SO3& offset_R_L_I = SO3()) (const vect3& offset_T_L_I = vect3()) (const vect3& vel = vect3()) (const vect3& bg = vect3()) (const vect3& ba = vect3()) (const S2& grav = S2()) ) : BOOST_PP_SEQ_ENUM_8  (pos(pos)) (rot(rot)) (offset_R_L_I(offset_R_L_I)) (offset_T_L_I(offset_T_L_I)) (vel(vel)) (bg(bg)) (ba(ba)) (grav(grav))
  state_ikfom(const vect3 &pos = vect3( ), const SO3 &rot = SO3( ), const SO3 &offset_R_L_I = SO3( ), const vect3 &offset_T_L_I = vect3( ), const vect3 &vel = vect3( ), const vect3 &bg = vect3( ), const vect3 &ba = vect3( ), const S2 &grav = S2( )) :
    pos(pos), rot(rot), offset_R_L_I(offset_R_L_I), offset_T_L_I(offset_T_L_I), vel(vel), bg(bg), ba(ba), grav(grav)
  {
  }
  int getDOF( ) const
  {
    return DOF;
  }
  void boxplus(const MTK::vectview<const scalar, DOF> &__vec, scalar __scale = 1)
  {
    // MTK_BOXPLUS (vect3, pos)  MTK_BOXPLUS (SO3, rot)  MTK_BOXPLUS (SO3, offset_R_L_I)  MTK_BOXPLUS (vect3, offset_T_L_I)  MTK_BOXPLUS (vect3, vel)  MTK_BOXPLUS (vect3, bg)  MTK_BOXPLUS (vect3, ba)  MTK_BOXPLUS (S2, grav)
    pos.boxplus(MTK::subvector(__vec, &self::pos), __scale);
    rot.boxplus(MTK::subvector(__vec, &self::rot), __scale);
    offset_R_L_I.boxplus(MTK::subvector(__vec, &self::offset_R_L_I), __scale);
    offset_T_L_I.boxplus(MTK::subvector(__vec, &self::offset_T_L_I), __scale);
    vel.boxplus(MTK::subvector(__vec, &self::vel), __scale);
    bg.boxplus(MTK::subvector(__vec, &self::bg), __scale);
    ba.boxplus(MTK::subvector(__vec, &self::ba), __scale);
    grav.boxplus(MTK::subvector(__vec, &self::grav), __scale);
  }
  void oplus(const MTK::vectview<const scalar, DIM> &__vec, scalar __scale = 1)
  {
    // MTK_OPLUS (vect3, pos)  MTK_OPLUS (SO3, rot)  MTK_OPLUS (SO3, offset_R_L_I)  MTK_OPLUS (vect3, offset_T_L_I)  MTK_OPLUS (vect3, vel)  MTK_OPLUS (vect3, bg)  MTK_OPLUS (vect3, ba)  MTK_OPLUS (S2, grav)
    pos.oplus(MTK::subvector_(__vec, &self::pos), __scale);
    rot.oplus(MTK::subvector_(__vec, &self::rot), __scale);
    offset_R_L_I.oplus(MTK::subvector_(__vec, &self::offset_R_L_I), __scale);
    offset_T_L_I.oplus(MTK::subvector_(__vec, &self::offset_T_L_I), __scale);
    vel.oplus(MTK::subvector_(__vec, &self::vel), __scale);
    bg.oplus(MTK::subvector_(__vec, &self::bg), __scale);
    ba.oplus(MTK::subvector_(__vec, &self::ba), __scale);
    grav.oplus(MTK::subvector_(__vec, &self::grav), __scale);
  }
  void boxminus(MTK::vectview<scalar, DOF> __res, const state_ikfom &__oth) const
  {
    // MTK_BOXMINUS (vect3, pos)  MTK_BOXMINUS (SO3, rot)  MTK_BOXMINUS (SO3, offset_R_L_I)  MTK_BOXMINUS (vect3, offset_T_L_I)  MTK_BOXMINUS (vect3, vel)  MTK_BOXMINUS (vect3, bg)  MTK_BOXMINUS (vect3, ba)  MTK_BOXMINUS (S2, grav)
    pos.boxminus(MTK::subvector(__res, &self::pos), __oth.pos);
    rot.boxminus(MTK::subvector(__res, &self::rot), __oth.rot);
    offset_R_L_I.boxminus(MTK::subvector(__res, &self::offset_R_L_I), __oth.offset_R_L_I);
    offset_T_L_I.boxminus(MTK::subvector(__res, &self::offset_T_L_I), __oth.offset_T_L_I);
    vel.boxminus(MTK::subvector(__res, &self::vel), __oth.vel);
    bg.boxminus(MTK::subvector(__res, &self::bg), __oth.bg);
    ba.boxminus(MTK::subvector(__res, &self::ba), __oth.ba);
    grav.boxminus(MTK::subvector(__res, &self::grav), __oth.grav);
  }
  friend std::ostream &operator<<(std::ostream &__os, const state_ikfom &__var)
  {
    // return __os MTK_OSTREAM (vect3, pos)  MTK_OSTREAM (SO3, rot)  MTK_OSTREAM (SO3, offset_R_L_I)  MTK_OSTREAM (vect3, offset_T_L_I)  MTK_OSTREAM (vect3, vel)  MTK_OSTREAM (vect3, bg)  MTK_OSTREAM (vect3, ba)  MTK_OSTREAM (S2, grav)   ;
    return __os << __var.pos << " " << __var.rot << " " << __var.offset_R_L_I << " " << __var.offset_T_L_I << " " << __var.vel << " " << __var.bg << " " << __var.ba << " " << __var.grav << " ";
  }
  void build_S2_state( )
  {
    // MTK_S2_state (vect3, pos)  MTK_S2_state (SO3, rot)  MTK_S2_state (SO3, offset_R_L_I)  MTK_S2_state (vect3, offset_T_L_I)  MTK_S2_state (vect3, vel)  MTK_S2_state (vect3, bg)  MTK_S2_state (vect3, ba)  MTK_S2_state (S2, grav)
    if (pos.TYP == 1)
    {
      S2_state.push_back(std::make_pair(pos.IDX, pos.DIM));
    }
    if (rot.TYP == 1)
    {
      S2_state.push_back(std::make_pair(rot.IDX, rot.DIM));
    }
    if (offset_R_L_I.TYP == 1)
    {
      S2_state.push_back(std::make_pair(offset_R_L_I.IDX, offset_R_L_I.DIM));
    }
    if (offset_T_L_I.TYP == 1)
    {
      S2_state.push_back(std::make_pair(offset_T_L_I.IDX, offset_T_L_I.DIM));
    }
    if (vel.TYP == 1)
    {
      S2_state.push_back(std::make_pair(vel.IDX, vel.DIM));
    }
    if (bg.TYP == 1)
    {
      S2_state.push_back(std::make_pair(bg.IDX, bg.DIM));
    }
    if (ba.TYP == 1)
    {
      S2_state.push_back(std::make_pair(ba.IDX, ba.DIM));
    }
    if (grav.TYP == 1)
    {
      S2_state.push_back(std::make_pair(grav.IDX, grav.DIM));
    }
  }
  void build_vect_state( )
  {
    // MTK_vect_state (vect3, pos)  MTK_vect_state (SO3, rot)  MTK_vect_state (SO3, offset_R_L_I)  MTK_vect_state (vect3, offset_T_L_I)  MTK_vect_state (vect3, vel)  MTK_vect_state (vect3, bg)  MTK_vect_state (vect3, ba)  MTK_vect_state (S2, grav)
    if (pos.TYP == 0)
    {
      (vect_state).push_back(std::make_pair(std::make_pair(pos.IDX, pos.DIM), vect3::DOF));
    }
    if (rot.TYP == 0)
    {
      (vect_state).push_back(std::make_pair(std::make_pair(rot.IDX, rot.DIM), SO3::DOF));
    }
    if (offset_R_L_I.TYP == 0)
    {
      (vect_state).push_back(std::make_pair(std::make_pair(offset_R_L_I.IDX, offset_R_L_I.DIM), SO3::DOF));
    }
    if (offset_T_L_I.TYP == 0)
    {
      (vect_state).push_back(std::make_pair(std::make_pair(offset_T_L_I.IDX, offset_T_L_I.DIM), vect3::DOF));
    }
    if (vel.TYP == 0)
    {
      (vect_state).push_back(std::make_pair(std::make_pair(vel.IDX, vel.DIM), vect3::DOF));
    }
    if (bg.TYP == 0)
    {
      (vect_state).push_back(std::make_pair(std::make_pair(bg.IDX, bg.DIM), vect3::DOF));
    }
    if (ba.TYP == 0)
    {
      (vect_state).push_back(std::make_pair(std::make_pair(ba.IDX, ba.DIM), vect3::DOF));
    }
    if (grav.TYP == 0)
    {
      (vect_state).push_back(std::make_pair(std::make_pair(grav.IDX, grav.DIM), S2::DOF));
    }
  }
  void build_SO3_state( )
  {
    // MTK_SO3_state (vect3, pos)  MTK_SO3_state (SO3, rot)  MTK_SO3_state (SO3, offset_R_L_I)  MTK_SO3_state (vect3, offset_T_L_I)  MTK_SO3_state (vect3, vel)  MTK_SO3_state (vect3, bg)  MTK_SO3_state (vect3, ba)  MTK_SO3_state (S2, grav)
    if (pos.TYP == 2)
    {
      (SO3_state).push_back(std::make_pair(pos.IDX, pos.DIM));
    }
    if (rot.TYP == 2)
    {
      (SO3_state).push_back(std::make_pair(rot.IDX, rot.DIM));
    }
    if (offset_R_L_I.TYP == 2)
    {
      (SO3_state).push_back(std::make_pair(offset_R_L_I.IDX, offset_R_L_I.DIM));
    }
    if (offset_T_L_I.TYP == 2)
    {
      (SO3_state).push_back(std::make_pair(offset_T_L_I.IDX, offset_T_L_I.DIM));
    }
    if (vel.TYP == 2)
    {
      (SO3_state).push_back(std::make_pair(vel.IDX, vel.DIM));
    }
    if (bg.TYP == 2)
    {
      (SO3_state).push_back(std::make_pair(bg.IDX, bg.DIM));
    }
    if (ba.TYP == 2)
    {
      (SO3_state).push_back(std::make_pair(ba.IDX, ba.DIM));
    }
    if (grav.TYP == 2)
    {
      (SO3_state).push_back(std::make_pair(grav.IDX, grav.DIM));
    }
  }
  void S2_hat(Eigen::Matrix<scalar, 3, 3> &res, int idx)
  {
    // MTK_S2_hat (vect3, pos)  MTK_S2_hat (SO3, rot)  MTK_S2_hat (SO3, offset_R_L_I)  MTK_S2_hat (vect3, offset_T_L_I)  MTK_S2_hat (vect3, vel)  MTK_S2_hat (vect3, bg)  MTK_S2_hat (vect3, ba)  MTK_S2_hat (S2, grav)
    if (pos.IDX == idx)
    {
      pos.S2_hat(res);
    }
    if (rot.IDX == idx)
    {
      rot.S2_hat(res);
    }
    if (offset_R_L_I.IDX == idx)
    {
      offset_R_L_I.S2_hat(res);
    }
    if (offset_T_L_I.IDX == idx)
    {
      offset_T_L_I.S2_hat(res);
    }
    if (vel.IDX == idx)
    {
      vel.S2_hat(res);
    }
    if (bg.IDX == idx)
    {
      bg.S2_hat(res);
    }
    if (ba.IDX == idx)
    {
      ba.S2_hat(res);
    }
    if (grav.IDX == idx)
    {
      grav.S2_hat(res);
    }
  }
  void S2_Nx_yy(Eigen::Matrix<scalar, 2, 3> &res, int idx)
  {
    // MTK_S2_Nx_yy (vect3, pos)  MTK_S2_Nx_yy (SO3, rot)  MTK_S2_Nx_yy (SO3, offset_R_L_I)  MTK_S2_Nx_yy (vect3, offset_T_L_I)  MTK_S2_Nx_yy (vect3, vel)  MTK_S2_Nx_yy (vect3, bg)  MTK_S2_Nx_yy (vect3, ba)  MTK_S2_Nx_yy (S2, grav)
    if (pos.IDX == idx)
    {
      pos.S2_Nx_yy(res);
    }
    if (rot.IDX == idx)
    {
      rot.S2_Nx_yy(res);
    }
    if (offset_R_L_I.IDX == idx)
    {
      offset_R_L_I.S2_Nx_yy(res);
    }
    if (offset_T_L_I.IDX == idx)
    {
      offset_T_L_I.S2_Nx_yy(res);
    }
    if (vel.IDX == idx)
    {
      vel.S2_Nx_yy(res);
    }
    if (bg.IDX == idx)
    {
      bg.S2_Nx_yy(res);
    }
    if (ba.IDX == idx)
    {
      ba.S2_Nx_yy(res);
    }
    if (grav.IDX == idx)
    {
      grav.S2_Nx_yy(res);
    }
  }
  void S2_Mx(Eigen::Matrix<scalar, 3, 2> &res, Eigen::Matrix<scalar, 2, 1> dx, int idx)
  {
    // MTK_S2_Mx (vect3, pos)  MTK_S2_Mx (SO3, rot)  MTK_S2_Mx (SO3, offset_R_L_I)  MTK_S2_Mx (vect3, offset_T_L_I)  MTK_S2_Mx (vect3, vel)  MTK_S2_Mx (vect3, bg)  MTK_S2_Mx (vect3, ba)  MTK_S2_Mx (S2, grav)
    if (pos.IDX == idx)
    {
      pos.S2_Mx(res, dx);
    }
    if (rot.IDX == idx)
    {
      rot.S2_Mx(res, dx);
    }
    if (offset_R_L_I.IDX == idx)
    {
      offset_R_L_I.S2_Mx(res, dx);
    }
    if (offset_T_L_I.IDX == idx)
    {
      offset_T_L_I.S2_Mx(res, dx);
    }
    if (vel.IDX == idx)
    {
      vel.S2_Mx(res, dx);
    }
    if (bg.IDX == idx)
    {
      bg.S2_Mx(res, dx);
    }
    if (ba.IDX == idx)
    {
      ba.S2_Mx(res, dx);
    }
    if (grav.IDX == idx)
    {
      grav.S2_Mx(res, dx);
    }
  }
  friend std::istream &operator>>(std::istream &__is, state_ikfom &__var)
  {
    // return __is MTK_ISTREAM (vect3, pos)  MTK_ISTREAM (SO3, rot)  MTK_ISTREAM (SO3, offset_R_L_I)  MTK_ISTREAM (vect3, offset_T_L_I)  MTK_ISTREAM (vect3, vel)  MTK_ISTREAM (vect3, bg)  MTK_ISTREAM (vect3, ba)  MTK_ISTREAM (S2, grav)   ;
    return __is >> __var.pos >> __var.rot >> __var.offset_R_L_I >> __var.offset_T_L_I >> __var.vel >> __var.bg >> __var.ba >> __var.grav;
  }
};


/********************input_ikfom***********************/
// 定义的输入状态
MTK_BUILD_MANIFOLD(input_ikfom,
                   ((vect3, acc))((vect3, gyro)));
struct input_ikfom
{
  typedef input_ikfom self;
  std::vector<std::pair<int, int>> S2_state;
  std::vector<std::pair<int, int>> SO3_state;
  std::vector<std::pair<std::pair<int, int>, int>> vect_state;
  // MTK_ENTRIES_OUTPUT_I ( 2, (vect3, acc) , ((vect3, gyro)) (~), 0, 0, S2_state, SO3_state )
  MTK::SubManifold<vect3, 0, 0> acc;
  // MTK_ENTRIES_OUTPUT_I ( 1, (vect3, gyro) ,  (~), 0 + BOOST_PP_TUPLE_ELEM_2_0 (vect3, acc)::DOF, 0 + BOOST_PP_TUPLE_ELEM_2_0 (vect3, acc)::DIM, S2_state, SO3_state)
  MTK::SubManifold<vect3, 0 + vect3::DOF, 0 + vect3::DIM> gyro;
  enum
  {
    DOF = vect3::DOF + 0 + vect3::DOF
  };
  enum
  {
    DIM = vect3::DIM + 0 + vect3::DIM
  };

  typedef vect3::scalar scalar;

  // input_ikfom ( BOOST_PP_SEQ_ENUM_2  (const vect3& acc = vect3()) (const vect3& gyro = vect3()) ) : BOOST_PP_SEQ_ENUM_2  (acc(acc)) (gyro(gyro))
  input_ikfom(const vect3 &acc = vect3( ), const vect3 &gyro = vect3( )) :
    acc(acc), gyro(gyro)
  {
  }
  int getDOF( ) const
  {
    return DOF;
  }
  void boxplus(const MTK::vectview<const scalar, DOF> &__vec, scalar __scale = 1)
  {
    // MTK_BOXPLUS(vect3, acc)  MTK_BOXPLUS(vect3, gyro)
    acc.boxplus(MTK::subvector(__vec, &self::acc), __scale);
    gyro.boxplus(MTK::subvector(__vec, &self::gyro), __scale);
  }
  void oplus(const MTK::vectview<const scalar, DIM> &__vec, scalar __scale = 1)
  {
    // MTK_OPLUS(vect3, acc)  MTK_OPLUS(vect3, gyro)
    acc.oplus(MTK::subvector_(__vec, &self::acc), __scale);
    gyro.oplus(MTK::subvector_(__vec, &self::gyro), __scale);
  }
  void boxminus(MTK::vectview<scalar, DOF> __res, const input_ikfom &__oth) const
  {
    // MTK_BOXMINUS(vect3, acc)  MTK_BOXMINUS(vect3, gyro)
    acc.boxminus(MTK::subvector(__res, &self::acc), __oth.acc);
    gyro.boxminus(MTK::subvector(__res, &self::gyro), __oth.gyro);
  }
  friend std::ostream &operator<<(std::ostream &__os, const input_ikfom &__var)
  {
    // return __os MTK_OSTREAM(vect3, acc)  MTK_OSTREAM(vect3, gyro);
    return __os << __var.acc << " " << __var.gyro << " ";
  }
  void build_S2_state( )
  {
    // MTK_S2_state(vect3, acc)  MTK_S2_state(vect3, gyro)
    if (acc.TYP == 1)
    {
      S2_state.push_back(std::make_pair(acc.IDX, acc.DIM));
    }
    if (gyro.TYP == 1)
    {
      S2_state.push_back(std::make_pair(gyro.IDX, gyro.DIM));
    }
  }
  void build_vect_state( )
  {
    // MTK_vect_state(vect3, acc)  MTK_vect_state(vect3, gyro)
    if (acc.TYP == 0)
    {
      (vect_state).push_back(std::make_pair(std::make_pair(acc.IDX, acc.DIM), vect3::DOF));
    }
    if (gyro.TYP == 0)
    {
      (vect_state).push_back(std::make_pair(std::make_pair(gyro.IDX, gyro.DIM), vect3::DOF));
    }
  }
  void build_SO3_state( )
  {
    // MTK_SO3_state(vect3, acc)  MTK_SO3_state(vect3, gyro)
    if (acc.TYP == 2)
    {
      (SO3_state).push_back(std::make_pair(acc.IDX, acc.DIM));
    }
    if (gyro.TYP == 2)
    {
      (SO3_state).push_back(std::make_pair(gyro.IDX, gyro.DIM));
    }
  }
  void S2_hat(Eigen::Matrix<scalar, 3, 3> &res, int idx)
  {
    // MTK_S2_hat(vect3, acc)  MTK_S2_hat(vect3, gyro)
    if (acc.IDX == idx)
    {
      acc.S2_hat(res);
    }
    if (gyro.IDX == idx)
    {
      gyro.S2_hat(res);
    }
  }
  void S2_Nx_yy(Eigen::Matrix<scalar, 2, 3> &res, int idx)
  {
    // MTK_S2_Nx_yy(vect3, acc)  MTK_S2_Nx_yy(vect3, gyro)
    if (acc.IDX == idx)
    {
      acc.S2_Nx_yy(res);
    }
    if (gyro.IDX == idx)
    {
      gyro.S2_Nx_yy(res);
    }
  }
  void S2_Mx(Eigen::Matrix<scalar, 3, 2> &res, Eigen::Matrix<scalar, 2, 1> dx, int idx)
  {
    // MTK_S2_Mx(vect3, acc)  MTK_S2_Mx(vect3, gyro)
    if (acc.IDX == idx)
    {
      acc.S2_Mx(res, dx);
    }
    if (gyro.IDX == idx)
    {
      gyro.S2_Mx(res, dx);
    }
  }
  friend std::istream &operator>>(std::istream &__is, input_ikfom &__var)
  {
    // return __is MTK_ISTREAM(vect3, acc)  MTK_ISTREAM(vect3, gyro);
    return __is >> __var.acc >> __var.gyro;
  }
};

/********************input_ikfom***********************/
// 定义的协方差噪声格式
// 角速度(3),加速度(3),角速度偏置(3),加速度偏置(3)
MTK_BUILD_MANIFOLD(process_noise_ikfom,
                   ((vect3, ng))((vect3, na))((vect3, nbg))((vect3, nba)));
struct process_noise_ikfom
{
  typedef process_noise_ikfom self;
  std::vector<std::pair<int, int>> S2_state;
  std::vector<std::pair<int, int>> SO3_state;
  std::vector<std::pair<std::pair<int, int>, int>> vect_state;
  // MTK_ENTRIES_OUTPUT_I ( 4, (vect3, ng) , ((vect3, na)) ((vect3, nbg)) ((vect3, nba)) (~), 0, 0, S2_state, SO3_state )
  MTK::SubManifold<vect3, 0, 0> ng;
  // MTK_ENTRIES_OUTPUT_I ( 3, (vect3, na) ,  ((vect3, nbg)) ((vect3, nba)) (~), 0 + BOOST_PP_TUPLE_ELEM_2_0 (vect3, ng)::DOF, 0 + BOOST_PP_TUPLE_ELEM_2_0 (vect3, ng)::DIM, S2_state, SO3_state)
  MTK::SubManifold<vect3, 0 + vect3::DOF, 0 + vect3::DIM> na;
  // MTK_ENTRIES_OUTPUT_I ( 2, (vect3, nbg) ,  ((vect3, nba)) (~), 0 + vect3::DOF + BOOST_PP_TUPLE_ELEM_2_0 (vect3, na)::DOF, 0 + vect3::DIM + BOOST_PP_TUPLE_ELEM_2_0 (vect3, na)::DIM, S2_state, SO3_state)
  MTK::SubManifold<vect3, 0 + vect3::DOF + vect3::DOF, 0 + vect3::DIM + vect3::DIM> nbg;

  // MTK_ENTRIES_OUTPUT_I ( 1, (vect3, nba) ,  (~), 0 + vect3::DOF + vect3::DOF + BOOST_PP_TUPLE_ELEM_2_0 (vect3, nbg)::DOF, 0 + vect3::DIM + vect3::DIM + BOOST_PP_TUPLE_ELEM_2_0 (vect3, nbg)::DIM, S2_state, SO3_state)
  MTK::SubManifold<vect3, 0 + vect3::DOF + vect3::DOF + vect3::DOF, 0 + vect3::DIM + vect3::DIM + vect3::DIM> nba;
  enum
  {
    DOF = vect3::DOF + 0 + vect3::DOF + vect3::DOF + vect3::DOF
  };
  enum
  {
    DIM = vect3::DIM + 0 + vect3::DIM + vect3::DIM + vect3::DIM
  };
  typedef vect3::scalar scalar;

  // process_noise_ikfom ( BOOST_PP_SEQ_ENUM_4  (const vect3& ng = vect3()) (const vect3& na = vect3()) (const vect3& nbg = vect3()) (const vect3& nba = vect3()) ) : BOOST_PP_SEQ_ENUM_4  (ng(ng)) (na(na)) (nbg(nbg)) (nba(nba))
  process_noise_ikfom(const vect3 &ng = vect3( ), const vect3 &na = vect3( ), const vect3 &nbg = vect3( ), const vect3 &nba = vect3( )) :
    ng(ng), na(na), nbg(nbg), nba(nba)
  {
  }
  int getDOF( ) const
  {
    return DOF;
  }
  void boxplus(const MTK::vectview<const scalar, DOF> &__vec, scalar __scale = 1)
  {
    // MTK_BOXPLUS (vect3, ng)  MTK_BOXPLUS (vect3, na)  MTK_BOXPLUS (vect3, nbg)  MTK_BOXPLUS (vect3, nba)
    ng.boxplus(MTK::subvector(__vec, &self::ng), __scale);
    na.boxplus(MTK::subvector(__vec, &self::na), __scale);
    nbg.boxplus(MTK::subvector(__vec, &self::nbg), __scale);
    nba.boxplus(MTK::subvector(__vec, &self::nba), __scale);
  }
  void oplus(const MTK::vectview<const scalar, DIM> &__vec, scalar __scale = 1)
  {
    // MTK_OPLUS (vect3, ng)  MTK_OPLUS (vect3, na)  MTK_OPLUS (vect3, nbg)  MTK_OPLUS (vect3, nba)
    ng.oplus(MTK::subvector_(__vec, &self::ng), __scale);
    na.oplus(MTK::subvector_(__vec, &self::na), __scale);
    nbg.oplus(MTK::subvector_(__vec, &self::nbg), __scale);
    nba.oplus(MTK::subvector_(__vec, &self::nba), __scale);
  }
  void boxminus(MTK::vectview<scalar, DOF> __res, const process_noise_ikfom &__oth) const
  {
    // MTK_BOXMINUS (vect3, ng)  MTK_BOXMINUS (vect3, na)  MTK_BOXMINUS (vect3, nbg)  MTK_BOXMINUS (vect3, nba)
    ng.boxminus(MTK::subvector(__res, &self::ng), __oth.ng);
    na.boxminus(MTK::subvector(__res, &self::na), __oth.na);
    nbg.boxminus(MTK::subvector(__res, &self::nbg), __oth.nbg);
    nba.boxminus(MTK::subvector(__res, &self::nba), __oth.nba);
  }
  friend std::ostream &operator<<(std::ostream &__os, const process_noise_ikfom &__var)
  {
    // return __os MTK_OSTREAM (vect3, ng)  MTK_OSTREAM (vect3, na)  MTK_OSTREAM (vect3, nbg)  MTK_OSTREAM (vect3, nba)   ;
    return __os << __var.ng << " " << __var.na << " " << __var.nbg << " " << __var.nba << " ";
  }
  void build_S2_state( )
  {
    // MTK_S2_state (vect3, ng)  MTK_S2_state (vect3, na)  MTK_S2_state (vect3, nbg)  MTK_S2_state (vect3, nba)
    if (ng.TYP == 1)
    {
      S2_state.push_back(std::make_pair(ng.IDX, ng.DIM));
    }
    if (na.TYP == 1)
    {
      S2_state.push_back(std::make_pair(na.IDX, na.DIM));
    }
    if (nbg.TYP == 1)
    {
      S2_state.push_back(std::make_pair(nbg.IDX, nbg.DIM));
    }
    if (nba.TYP == 1)
    {
      S2_state.push_back(std::make_pair(nba.IDX, nba.DIM));
    }
  }
  void build_vect_state( )
  {
    // MTK_vect_state (vect3, ng)  MTK_vect_state (vect3, na)  MTK_vect_state (vect3, nbg)  MTK_vect_state (vect3, nba)
    if (ng.TYP == 0)
    {
      (vect_state).push_back(std::make_pair(std::make_pair(ng.IDX, ng.DIM), vect3::DOF));
    }
    if (na.TYP == 0)
    {
      (vect_state).push_back(std::make_pair(std::make_pair(na.IDX, na.DIM), vect3::DOF));
    }
    if (nbg.TYP == 0)
    {
      (vect_state).push_back(std::make_pair(std::make_pair(nbg.IDX, nbg.DIM), vect3::DOF));
    }
    if (nba.TYP == 0)
    {
      (vect_state).push_back(std::make_pair(std::make_pair(nba.IDX, nba.DIM), vect3::DOF));
    }
  }
  void build_SO3_state( )
  {
    // MTK_SO3_state (vect3, ng)  MTK_SO3_state (vect3, na)  MTK_SO3_state (vect3, nbg)  MTK_SO3_state (vect3, nba)
    if (ng.TYP == 2)
    {
      (SO3_state).push_back(std::make_pair(ng.IDX, ng.DIM));
    }
    if (na.TYP == 2)
    {
      (SO3_state).push_back(std::make_pair(na.IDX, na.DIM));
    }
    if (nbg.TYP == 2)
    {
      (SO3_state).push_back(std::make_pair(nbg.IDX, nbg.DIM));
    }
    if (nba.TYP == 2)
    {
      (SO3_state).push_back(std::make_pair(nba.IDX, nba.DIM));
    }
  }
  void S2_hat(Eigen::Matrix<scalar, 3, 3> &res, int idx)
  {
    // MTK_S2_hat (vect3, ng)  MTK_S2_hat (vect3, na)  MTK_S2_hat (vect3, nbg)  MTK_S2_hat (vect3, nba)
    if (ng.IDX == idx)
    {
      ng.S2_hat(res);
    }
    if (na.IDX == idx)
    {
      na.S2_hat(res);
    }
    if (nbg.IDX == idx)
    {
      nbg.S2_hat(res);
    }
    if (nba.IDX == idx)
    {
      nba.S2_hat(res);
    }
  }
  void S2_Nx_yy(Eigen::Matrix<scalar, 2, 3> &res, int idx)
  {
    // MTK_S2_Nx_yy (vect3, ng)  MTK_S2_Nx_yy (vect3, na)  MTK_S2_Nx_yy (vect3, nbg)  MTK_S2_Nx_yy (vect3, nba)
    if (ng.IDX == idx)
    {
      ng.S2_Nx_yy(res);
    }
    if (na.IDX == idx)
    {
      na.S2_Nx_yy(res);
    }
    if (nbg.IDX == idx)
    {
      nbg.S2_Nx_yy(res);
    }
    if (nba.IDX == idx)
    {
      nba.S2_Nx_yy(res);
    }
  }
  void S2_Mx(Eigen::Matrix<scalar, 3, 2> &res, Eigen::Matrix<scalar, 2, 1> dx, int idx)
  {
    // MTK_S2_Mx (vect3, ng)  MTK_S2_Mx (vect3, na)  MTK_S2_Mx (vect3, nbg)  MTK_S2_Mx (vect3, nba)
    if (ng.IDX == idx)
    {
      ng.S2_Mx(res, dx);
    }
    if (na.IDX == idx)
    {
      na.S2_Mx(res, dx);
    }
    if (nbg.IDX == idx)
    {
      nbg.S2_Mx(res, dx);
    }
    if (nba.IDX == idx)
    {
      nba.S2_Mx(res, dx);
    }
  }
  friend std::istream &operator>>(std::istream &__is, process_noise_ikfom &__var)
  {
    // return __is MTK_ISTREAM (vect3, ng)  MTK_ISTREAM (vect3, na)  MTK_ISTREAM (vect3, nbg)  MTK_ISTREAM (vect3, nba)   ;
    return __is >> __var.ng >> __var.na >> __var.nbg >> __var.nba;
  }
};

#endif