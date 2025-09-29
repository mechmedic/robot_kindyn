(* ::Package:: *)

(* ::Title:: *)
(*Mathematica Package For Robotics*)


(* ::Section:: *)
(*Public Function Declaration*)


BeginPackage["ChunRobotics`"];


VecToso3::usage="VecToso3[w_]. Convert 3x1 vector to 3x3 skew symmetric matrix so3";


so3ToVec::usage="so3ToVec[what_]. Convert 3x3 skew symmetric matrix so3 to 3x1 vector";


VecTose3::usage="VecTose3[w_]. Convert 6x1 twist vector to 4x4 twist matrix se3";


se3ToVec::usage="se3ToVec[what_]. Convert 4x4 twist symmetric matrix se3 to 6x1 twist vector";


RpToTrans::usage="RpToTrans[R_,p_]. Creates 4x4 transformation matrix from rotation matrix R and 3x1 vector p";


TransToR::usage="TransToR[T_]. Extracts rotation matrix part of the 4x4 transformation matrix";


TransToP::usage="TransToP[T_]. Extracts 3x1 translational part of the 4x4 transformation matrix";


InvTrans::usage="InvTrans[T_]. Calculates inverse of the transformation matrix";


MatrixExp3::usage="MatrixExp3[what_]. Evaluates matrix exponential of 3x3 skew symmetric matrix";


MatrixExp6::usage="MatrixExp6[xihat_]. Evaluates matrix exponential of 4x4 twist matrix";


Adjoint::usage="Adjoint[T_]. Generates 6x6 Adjoint transformation matrix from 4x4 transformation matrix";


LieBracket::usage="LieBracket[xi_]. Generates 6x6 adjoint transformation matrix from 6x1 twist vector";


FwdKin::usage="FwdKin[xi_,th_,T0_]. Evaluates forward kinematics from twist set 'xi', joint angle 'th' and initial configuration 'T0'";


SpatialJacobian::usage="SpatialJacobian[xi_,th_,T0_]. Evaluates geometric spatial Jacobian from twist set 'xi', joint angle 'th' and initial configuration 'T0'";


BodyJacobian::usage="BodyJacobian[xi_,th_,T0_]. Evaluates geometric spatial Jacobian from twist set 'xi', joint angle 'th' and initial configuration 'T0'";


MassMatrix::usage="MassMatrix[xi_,th_,Tlist_,Mlist_]. Evaluates mass matrix of the dynamic equation from twist set 'xi', joint angle 'th', list of initial configuration of each links and mass matrix 'Tlist' and 'Mlist'";


CoriolisMatrix::usage="CoriolisMatrix[xi_,th_,thdot_,Tlist_,Mlist_]. Evaluates Coriolis matrix of the dynamic equation from twist set 'xi', joint angle 'th', velocity 'thdt', list of initial configuration of each links and mass matrix 'Tlist' and 'Mlist'";


GravityTerm::usage="GravityTerm[xi_,th_,Tlist_,Mlist_,g_]. Evaluates gravity vector of the dynamic equation from twist set 'xi', joint angle 'th', list of initial configuration of each links and mass matrix 'Tlist' and 'Mlist' and gravity vector 'g'";


(* End of Function Declaration *)


(* ::Section:: *)
(*Function Definitions*)


Begin["`Private`"];


VecToso3[w_]:={{0,-w[[3]],w[[2]]},{w[[3]],0,-w[[1]]},{-w[[2]],w[[1]],0}}


so3ToVec[what_]:={what[[3,2]],what[[1,3]],what[[2,1]]}


VecTose3[xi_]:={{0,-xi[[3]],xi[[2]],xi[[4]]},{xi[[3]],0,-xi[[1]],xi[[5]]},{-xi[[2]],xi[[1]],0,xi[[6]]},{0,0,0,0}}


se3ToVec[xihat_]:={xihat[[3,2]],xihat[[1,3]],xihat[[2,1]],xihat[[1,4]],xihat[[2,4]],xihat[[3,4]]}


RpToTrans[R_,p_]:= Join[Join[R,Transpose[{p}],2],{{0,0,0,1}}]


TransToR[T_]:=T[[1;;3,1;;3]]


TransToP[T_]:=T[[1;;3,4]]


InvTrans[T_]:=Module[{R,p},
	R=TransToR[T];p=TransToP[T];
	RpToTrans[Transpose[R],-Transpose[R].p]
	]


MatrixExp3[what_]:=Module[{th,wnorm},
	th=Norm[so3ToVec[what]]; 
	wnorm=what/th;
	Simplify[IdentityMatrix[3]+Sin[th]*wnorm+(1-Cos[th])*wnorm.wnorm]
	]


MatrixExp6[xihat_]:=Module[{th,w,v,I,R,p},
	w=se3ToVec[xihat][[1;;3]]; 
	v=se3ToVec[xihat][[4;;6]];
	I=IdentityMatrix[3];
	th=Norm[w]; 
	If[NumberQ[th] &&th==0,
		RpToTrans[I,v]
		, (* else *)
		w=w/th; v=v/th; R=MatrixExp3[VecToso3[w*th]]; 
		p=(I-R).Cross[w,v]+(w.v)*w*th;
		RpToTrans[R,p]
		]
	]


Adjoint[T_]:=Module[{R,p},
	R=TransToR[T];p=TransToP[T];
	Join[Join[R,ConstantArray[0,{3,3}],2],Join[VecToso3[p].R,R,2]]
	]


LieBracket[xi_]:=Module[{w,v},
	w=xi[[1;;3]];v=xi[[4;;6]];
	Join[Join[VecToso3[w],ConstantArray[0,{3,3}],2],Join[VecToso3[v],VecToso3[w],2]]
	]


FwdKin[xi_,th_,T0_]:=Module[{n,T,i},
	n=Length[th];T=IdentityMatrix[4];
	For[i=1,i<n+1,i++, T=T.MatrixExp6[VecTose3[xi[[i]]*th[[i]]]];];
	T.T0
	]


SpatialJacobian[xi_,th_,T0_]:=Module[{n,Js,i,j,T},
	n=Length[th];Js=ConstantArray[0,{6,n}];
	Js[[1;;6,1]]=xi[[1]];
	For[i=2,i<n+1,i++,T=IdentityMatrix[4];
		For[j= 1, j<i,j++, T=T.MatrixExp6[VecTose3[xi[[j]]*th[[j]]]]];
		Js[[1;;6,i]]=Adjoint[T].xi[[i]]];
	Js
	]


BodyJacobian[xi_,th_,T0_]:=Module[{n,i,Jb,T},
	n=Length[th]; Jb=ConstantArray[0,{6,n}];
	T=InvTrans[T0];
	Jb[[1;;6,n]]=Adjoint[T].xi[[n]];
	For[i=(n-1),i>0,i--,
		T=T.MatrixExp6[ -VecTose3[ xi[[i+1]]*th[[i+1]] ] ];
		Jb[[1;;6,i]]=Adjoint[T].xi[[i]];
	];
	Jb
	]


MassMatrix[xi_,th_,Tlist_,Mlist_]:=Module[{n,i,G,T,J,M},
	n=Length[th];M=ConstantArray[0,{n,n}];
	For[i=1, i<n+1,i++,
		T=Tlist[[i]]; G=Mlist[[i]];
		J=BodyJacobian[xi[[1;;i,All]],th[[1;;i]],T];
		J=Join[J,ConstantArray[0,{6,n-i}],2];
		M=M+(Transpose[J].G.J);
		];
	M
	]


Christoffel[M_,th_,i_,j_,k_]:=0.5*(D[M[[i,j]],th[[k]]]+D[M[[i,k]],th[[j]]]-D[M[[j,k]],th[[i]]])


CoriolisMatrix[xi_, th_, thdot_, Tlist_, Mlist_] := Module[{n, M, i, j, k, Cij, C},
  	n = Length[th]; C = ConstantArray[0, {n, n}];
  	M = MassMatrix[xi, th, Tlist, Mlist];
  	For[i = 1, i < n + 1, i++,
   		For[j = 1, j < n + 1, j++, Cij = 0;
    			For[k = 1, k < n + 1, k++, Cij = Cij + Christoffel[M, th, i, j, k]*thdot[[k]]];
    			C[[i, j]] = Cij;
              ];
    	  ];
   	C
   	];


GravityTerm[xi_,th_,Tlist_,Mlist_,g_]:=Module[{n,i,j,m,M,c,T0,T,h,PE,gterm},
	n=Length[th];gterm = ConstantArray[0,{n,1}];PE = 0;
	For[i=1, i<n+1,i++,T0=Tlist[[i]]; 
		T=IdentityMatrix[4]; 
		M=Mlist[[i]];
		m=M[[4,4]];
		c=Join[so3ToVec[M[[1;;3,4;;6]]],{m},1];
		For[j=1,j<i+1,j++,
			T=T.MatrixExp6[VecTose3[xi[[j]]*th[[j]]]];
			];
		h=T.T0.c;
		PE = PE+g.h[[1;;3]];
		];
	For[i=1, i<n+1,i++,
		gterm[[i]]=D[PE,th[[i]]];
		];
	gterm]


End[];


EndPackage[];
