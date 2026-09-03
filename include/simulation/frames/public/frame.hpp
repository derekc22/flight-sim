#pragma once
#include <unordered_set>
#include "simulation/constants/public/linalg.hpp"
#include "simulation/frames/public/data/types.hpp"

namespace frames {

    struct Frame {
        FrameID id;
        Frame* parent = nullptr;

        Frame(FrameID id, Frame* p);

        void _set(const dynamics::HomogeneousTransformationMatrix& H);
        void _set(const dynamics::OrientationMatrix& C);
        void _set(const dynamics::Position& p);
        void _set(const dynamics::OrientationQuaternion& q);
        void _set(const dynamics::EulerAngles& eul);
        void _set(const dynamics::OrientationMatrixRate& C_dot);
        void _set(const dynamics::OrientationQuaternionRate& q_dot);
        void _set(const dynamics::AngularVelocity& w);
        void _set(const dynamics::EulerAngleRates& eul_dot);
        void _set(const dynamics::AngularVelocityQuaternion& wq);
        void _set(const dynamics::TranslationalVelocity& v);
        void _set(const dynamics::Gravity& g);

        virtual MutableFrameView view() = 0;
        virtual FrameView view() const = 0;
        virtual ~Frame();

        void set(const SetOptions& opts);
        
        std::unordered_set<Frame*> dependents;
        std::unordered_set<Frame*> dependent_on;
        void add_as_direct_dependent(Frame* p);
    };

    struct ECEFFrame : Frame { 
        ECEFFrame();
        dynamics::HomogeneousTransformationMatrix H{ constants::HI };
        dynamics::OrientationQuaternion q{ constants::qI };
        dynamics::EulerAngles eul{ constants::Zero3 };
        dynamics::OrientationMatrixRate C_dot{ constants::Zero3x3 };
        dynamics::OrientationQuaternionRate q_dot{ constants::q0 };
        dynamics::AngularVelocity w{ constants::Zero3 };
        dynamics::EulerAngleRates eul_dot{ constants::Zero3 };
        dynamics::AngularVelocityQuaternion wq{ constants::q0 };
        dynamics::TranslationalVelocity v{ constants::Zero3 };
        dynamics::Gravity g{ constants::Zero3 };

        MutableFrameView view() override;
        FrameView view() const override;
    }; 

    // {ECEF} -> {NED}
    struct NEDFrameECEF : Frame {
        NEDFrameECEF(ECEFFrame* pECEFFrame);
        dynamics::HomogeneousTransformationMatrix HEN; 
        dynamics::OrientationQuaternion qEN;
        dynamics::EulerAngles eulEN;
        dynamics::OrientationMatrixRate CEN_dot;
        dynamics::OrientationQuaternionRate qEN_dot;
        dynamics::AngularVelocity wN_NE;
        dynamics::EulerAngleRates eulEN_dot;
        dynamics::AngularVelocityQuaternion wq_NE;
        dynamics::TranslationalVelocity vN_NE;
        dynamics::Gravity gN;

        MutableFrameView view() override;
        FrameView view() const override;
    };

    // {ECEF} -> {FRD}
    struct FRDFrameECEF : Frame {
        FRDFrameECEF(ECEFFrame* pECEFFrame);
        dynamics::HomogeneousTransformationMatrix HEB; 
        dynamics::OrientationQuaternion qEB;
        dynamics::EulerAngles eulEB;
        dynamics::OrientationMatrixRate CEB_dot;
        dynamics::OrientationQuaternionRate qEB_dot;
        dynamics::AngularVelocity wB_BE;
        dynamics::EulerAngleRates eulEB_dot;
        dynamics::AngularVelocityQuaternion wq_BE;
        dynamics::TranslationalVelocity vB_BE;
        dynamics::Gravity gB;

        MutableFrameView view() override;  
        FrameView view() const override;  
    };

    // {NED} -> {FRD}
    struct FRDFrameNED : Frame { 
        FRDFrameNED(NEDFrameECEF* pNEDFrameECEF);
        dynamics::HomogeneousTransformationMatrix HNB; 
        dynamics::OrientationQuaternion qNB;
        dynamics::EulerAngles eulNB;
        dynamics::OrientationMatrixRate CNB_dot;
        dynamics::OrientationQuaternionRate qNB_dot;
        dynamics::AngularVelocity wB_BN;
        dynamics::EulerAngleRates eulNB_dot;
        dynamics::AngularVelocityQuaternion wq_BN;
        dynamics::TranslationalVelocity vB_BN;
        dynamics::Gravity gB;

        MutableFrameView view() override; 
        FrameView view() const override; 
    };

    // {FRD} -> {CG}
    struct CGFrameFRD : Frame {
        CGFrameFRD(FRDFrameNED* pFRDFrameNED);
        dynamics::HomogeneousTransformationMatrix HBG; 
        dynamics::OrientationQuaternion qBG;
        dynamics::EulerAngles eulBG;
        dynamics::OrientationMatrixRate CBG_dot;
        dynamics::OrientationQuaternionRate qBG_dot;
        dynamics::AngularVelocity wG_GB;
        dynamics::EulerAngleRates eulBG_dot;
        dynamics::AngularVelocityQuaternion wq_GB;
        dynamics::TranslationalVelocity vG_GB;
        dynamics::Gravity gG;

        MutableFrameView view() override; 
        FrameView view() const override; 
    };

    // {FRD} -> {STAB}
    struct STABFrameFRD : Frame {
        STABFrameFRD(FRDFrameNED* pFRDFrameNED);
        dynamics::HomogeneousTransformationMatrix HBS; 
        dynamics::OrientationQuaternion qBS;
        dynamics::EulerAngles eulBS;
        dynamics::OrientationMatrixRate CBS_dot;
        dynamics::OrientationQuaternionRate qBS_dot;
        dynamics::AngularVelocity wS_SB;
        dynamics::EulerAngleRates eulBS_dot;
        dynamics::AngularVelocityQuaternion wq_SB;
        dynamics::TranslationalVelocity vS_SB;
        dynamics::Gravity gS;

        MutableFrameView view() override; 
        FrameView view() const override; 
    };

    // {STAB} -> {WIND}
    struct WINDFrameSTAB : Frame {
        WINDFrameSTAB(STABFrameFRD* pSTABFrameFRD);
        dynamics::HomogeneousTransformationMatrix HSW; 
        dynamics::OrientationQuaternion qSW;
        dynamics::EulerAngles eulSW;
        dynamics::OrientationMatrixRate CSW_dot;
        dynamics::OrientationQuaternionRate qSW_dot;
        dynamics::AngularVelocity wW_WS;
        dynamics::EulerAngleRates eulSW_dot;
        dynamics::AngularVelocityQuaternion wq_WS;
        dynamics::TranslationalVelocity vW_WS;
        dynamics::Gravity gW;

        MutableFrameView view() override; 
        FrameView view() const override; 
    };

}
