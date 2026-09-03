#include "simulation/frames/public/frame.hpp"
#include "simulation/transforms/public/detail/s3.hpp"
#include "simulation/dynamics/public/detail/derivatives.hpp"

namespace frames {
	MutableFrameView ECEFFrame::view()
	{
		return {&H, &q, &eul, &C_dot, &q_dot, &w, &eul_dot, &wq, &v, &g};
	}

	FrameView ECEFFrame::view() const
	{
		return {&H, &q, &eul, &C_dot, &q_dot, &w, &eul_dot, &wq, &v, &g};
	}

	MutableFrameView NEDFrameECEF::view()
	{
		return {&HEN, &qEN, &eulEN, &CEN_dot, &qEN_dot, &wN_NE, &eulEN_dot, &wq_NE, &vN_NE, &gN};
	}

	FrameView NEDFrameECEF::view() const
	{
		return {&HEN, &qEN, &eulEN, &CEN_dot, &qEN_dot, &wN_NE, &eulEN_dot, &wq_NE, &vN_NE, &gN};
	}

	MutableFrameView FRDFrameECEF::view()
	{
		return {&HEB, &qEB, &eulEB, &CEB_dot, &qEB_dot, &wB_BE, &eulEB_dot, &wq_BE, &vB_BE, &gB};
	}

	FrameView FRDFrameECEF::view() const
	{
		return {&HEB, &qEB, &eulEB, &CEB_dot, &qEB_dot, &wB_BE, &eulEB_dot, &wq_BE, &vB_BE, &gB};
	}

	MutableFrameView FRDFrameNED::view()
	{
		return {&HNB, &qNB, &eulNB, &CNB_dot, &qNB_dot, &wB_BN, &eulNB_dot, &wq_BN, &vB_BN, &gB};
	}

	FrameView FRDFrameNED::view() const
	{
		return {&HNB, &qNB, &eulNB, &CNB_dot, &qNB_dot, &wB_BN, &eulNB_dot, &wq_BN, &vB_BN, &gB};
	}

	MutableFrameView CGFrameFRD::view()
	{
		return {&HBG, &qBG, &eulBG, &CBG_dot, &qBG_dot, &wG_GB, &eulBG_dot, &wq_GB, &vG_GB, &gG};
	}

	FrameView CGFrameFRD::view() const
	{
		return {&HBG, &qBG, &eulBG, &CBG_dot, &qBG_dot, &wG_GB, &eulBG_dot, &wq_GB, &vG_GB, &gG};
	}

	MutableFrameView STABFrameFRD::view()
	{
		return {&HBS, &qBS, &eulBS, &CBS_dot, &qBS_dot, &wS_SB, &eulBS_dot, &wq_SB, &vS_SB, &gS};
	}

	FrameView STABFrameFRD::view() const
	{
		return {&HBS, &qBS, &eulBS, &CBS_dot, &qBS_dot, &wS_SB, &eulBS_dot, &wq_SB, &vS_SB, &gS};
	}

	MutableFrameView WINDFrameSTAB::view()
	{
		return {&HSW, &qSW, &eulSW, &CSW_dot, &qSW_dot, &wW_WS, &eulSW_dot, &wq_WS, &vW_WS, &gW};
	}

	FrameView WINDFrameSTAB::view() const
	{
		return {&HSW, &qSW, &eulSW, &CSW_dot, &qSW_dot, &wW_WS, &eulSW_dot, &wq_WS, &vW_WS, &gW};
	}

	void Frame::_set(
	    const dynamics::HomogeneousTransformationMatrix& H)
	{
		_set(H.C());
		_set(H.p());
	}

	void Frame::_set(
	    const dynamics::OrientationMatrix& C)
	{
		MutableFrameView mfv = view();
		mfv.H->set(C);
		mfv.q->set(C);
		mfv.eul->set(C);
	}

	void Frame::_set(
	    const dynamics::Position& p)
	{
		MutableFrameView mfv = view();
		mfv.H->set(p);
	}

	void Frame::_set(
	    const dynamics::OrientationQuaternion& q)
	{
		MutableFrameView mfv = view();
		dynamics::OrientationQuaternion q_{transforms::normalize_and_canonicalize(q.data)};
		*mfv.q = q_;
		mfv.H->set(q_);
		mfv.eul->set(q_);
	}

	void Frame::_set(
	    const dynamics::EulerAngles& eul)
	{
		MutableFrameView mfv = view();
		mfv.H->set(eul);
		mfv.q->set(eul);
		*mfv.eul = eul;
	}

	void Frame::_set(
	    const dynamics::OrientationMatrixRate& C_dot)
	{
		MutableFrameView mfv = view();
		*mfv.C_dot = C_dot;
		dynamics::AngularVelocity w = dynamics::CIB_dot_to_wB_BI(*mfv.C_dot, mfv.H->C());
		mfv.q_dot->set(*mfv.q, w);
		*mfv.w = w;
		mfv.eul_dot->set(w, *mfv.eul);
		mfv.wq->set(w);
	}

	void Frame::_set(
	    const dynamics::OrientationQuaternionRate& q_dot)
	{
		MutableFrameView mfv = view();
		mfv.C_dot->set(q_dot, *mfv.q, mfv.H->C());
		*mfv.q_dot = q_dot;
		dynamics::AngularVelocity w = dynamics::qIB_dot_to_wB_BI(q_dot, *mfv.q);
		*mfv.w = w;
		mfv.eul_dot->set(w, *mfv.eul);
		mfv.wq->set(w);
	}

	void Frame::_set(
	    const dynamics::AngularVelocity& w)
	{
		MutableFrameView mfv = view();
		mfv.C_dot->set(mfv.H->C(), w);
		mfv.q_dot->set(*mfv.q, w);
		*mfv.w = w;
		mfv.eul_dot->set(w, *mfv.eul);
		mfv.wq->set(w);
	}

	void Frame::_set(
	    const dynamics::EulerAngleRates& eul_dot)
	{
		MutableFrameView mfv = view();
		dynamics::AngularVelocity w = dynamics::eul_dot_to_wB_BI(eul_dot, *mfv.eul);
		mfv.C_dot->set(mfv.H->C(), w);
		mfv.q_dot->set(*mfv.q, w);
		*mfv.w = w;
		*mfv.eul_dot = eul_dot;
		mfv.wq->set(w);
	}

	void Frame::_set(
	    const dynamics::AngularVelocityQuaternion& wq)
	{
		MutableFrameView mfv = view();
		dynamics::AngularVelocity w = wq.w();
		mfv.C_dot->set(mfv.H->C(), w);
		mfv.q_dot->set(*mfv.q, w);
		*mfv.w = w;
		mfv.eul_dot->set(w, *mfv.eul);
		*mfv.wq = wq;
	}

	void Frame::_set(
	    const dynamics::TranslationalVelocity& v)
	{
		MutableFrameView mfv = view();
		*mfv.v = v;
	}

	void Frame::_set(
	    const dynamics::Gravity& g)
	{
		MutableFrameView mfv = view();
		*mfv.g = g;
	}

	Frame::Frame(FrameID id, Frame* p) : id(id), parent(p) {};

	Frame::~Frame()
	{
		// Destructor for frame A

		// Parent side cleanup (for each parent P in dependent_on)
		for (auto it = dependent_on.begin(); it != dependent_on.end();) {
			Frame* parent = *it;
			parent->dependents.erase(this); // Remove A from P’s dependents
			it = dependent_on.erase(it);    // Remove P from A’s dependent_on
		}
		// Child side (for each child C in dependents):
		for (auto it = dependents.begin(); it != dependents.end();) {
			Frame* dep = *it;
			dep->dependent_on.erase(this); // Remove A from C’s dependent_on
			it = dependents.erase(it);     // Remove C from A’s dependents
		}
	}

	void Frame::add_as_direct_dependent(
	    Frame* p)
	{
		dependents.insert(p);
		p->dependent_on.insert(this);
	}

	ECEFFrame::ECEFFrame() : Frame(FrameID::ECEFFrame, nullptr) {};
	NEDFrameECEF::NEDFrameECEF(ECEFFrame* pECEFFrame) : Frame(FrameID::NEDFrameECEF, pECEFFrame) {};
	FRDFrameECEF::FRDFrameECEF(ECEFFrame* pECEFFrame) : Frame(FrameID::FRDFrameECEF, pECEFFrame) {};
	FRDFrameNED::FRDFrameNED(NEDFrameECEF* pNEDFrameECEF) : Frame(FrameID::FRDFrameNED, pNEDFrameECEF) {};
	CGFrameFRD::CGFrameFRD(FRDFrameNED* pFRDFrameNED) : Frame(FrameID::CGFrameFRD, pFRDFrameNED) {};
	STABFrameFRD::STABFrameFRD(FRDFrameNED* pFRDFrameNED) : Frame(FrameID::STABFrameFRD, pFRDFrameNED) {};
	WINDFrameSTAB::WINDFrameSTAB(STABFrameFRD* pSTABFrameFRD) : Frame(FrameID::WINDFrameSTAB, pSTABFrameFRD) {};

	void Frame::set(
	    const SetOptions& opts)
	{
		if (opts.H.has_value()) {
			_set(*opts.H);
		}
		if (opts.C.has_value()) {
			_set(*opts.C);
		}
		if (opts.p.has_value()) {
			_set(*opts.p);
		}
		if (opts.q.has_value()) {
			_set(*opts.q);
		}
		if (opts.eul.has_value()) {
			_set(*opts.eul);
		}
		if (opts.C_dot.has_value()) {
			_set(*opts.C_dot);
		}
		if (opts.q_dot.has_value()) {
			_set(*opts.q_dot);
		}
		if (opts.w.has_value()) {
			_set(*opts.w);
		}
		if (opts.eul_dot.has_value()) {
			_set(*opts.eul_dot);
		}
		if (opts.wq.has_value()) {
			_set(*opts.wq);
		}
		if (opts.v.has_value()) {
			_set(*opts.v);
		}
		if (opts.g.has_value()) {
			_set(*opts.g);
		}
	}

} // namespace frames
