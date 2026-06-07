
#include <Eigen/Dense>
#include <format>
#include <string>
#include <iostream>
#include <stdexcept>
#include <queue>
#include <unordered_set>
#include "simulation/aerodynamics/public.hpp"
#include "simulation/atmospheric/public.hpp"
#include "simulation/dynamics/public.hpp"
#include "simulation/frames/public.hpp"
#include "simulation/geography/public.hpp"
#include "simulation/vehicles/public.hpp"
#include "simulation/constants/public.hpp"
#include "simulation/util/public.hpp"
#include "simulation/structural/public.hpp"
#include "simulation/control/public.hpp"
#include "simulation/estimation/public.hpp"
#include "simulation/actuators/public.hpp"
#include "simulation/operating/public.hpp"
#include "simulation/avionics/public.hpp"
#include "simulation/guidance/public.hpp"



namespace vehicles {

    void StepOptions::validate(const StepOptions& opts) {
        std::string err_msg = "vehicles::StepOptions::validate: Invalid input, cannot pass StepOptions for FRDFrameECEF and FRDFrameNED simultaneously";
        if (opts.FRDFrameECEFStepOpts.has_value() && opts.FRDFrameNEDStepOpts.has_value()) throw std::invalid_argument(err_msg);
    }

    void _StepOptions::validate(const frames::Frame& F, const _StepOptions& opts) {

        const bool has_H = opts.H.has_value();
        const bool has_C = opts.C.has_value();
        const bool has_p = opts.p.has_value();
        const bool has_q = opts.q.has_value();
        const bool has_eul = opts.eul.has_value(); 
        const bool has_C_dot = opts.C_dot.has_value();
        const bool has_q_dot = opts.q_dot.has_value();
        const bool has_w = opts.w.has_value();
        const bool has_eul_dot  = opts.eul_dot.has_value();
        const bool has_wq = opts.wq.has_value(); 
        const bool has_v = opts.v.has_value();
        const bool has_g = opts.g.has_value();
        const bool has_lat = opts.lat.has_value();
        const bool has_lon = opts.lon.has_value();
        const bool has_alt = opts.alt.has_value();
        const bool has_alpha = opts.alpha.has_value();
        const bool has_beta = opts.beta.has_value();
        const bool has_rbs = opts.rbs.has_value();
        const bool has_gps = opts.gps.has_value();
        const bool has_ads = opts.ads.has_value();

        // lat, lon, alt must be passed together
        const bool has_geo_any = (has_lat || has_lon || has_alt);
        const bool has_geo_all = (has_lat && has_lon && has_alt);

        if (has_geo_any && !has_geo_all) {
            std::string err_msg = std::format("vehicles::_StepOptions::validate: Invalid geographic input for {}, lat, lon, alt must be passed together", F.name);
            throw std::invalid_argument(err_msg);
        }

        // H cannot be combined with any other position or orientation representations
        if (has_H && (has_C || has_p || has_q || has_eul || has_geo_any || has_gps)) {
            std::string err_msg = std::format("vehicles::_StepOptions::validate: Invalid position or orientation input for {}, cannot pass C, p, q, eul, (lat, lon, alt), gps with H", F.name);
            throw std::invalid_argument(err_msg);
        }

        // Only one position representation at a time
        if ((has_p && has_geo_any) ||
            (has_p && has_gps) ||
            (has_geo_any && has_gps)) {
            std::string err_msg = std::format("vehicles::_StepOptions::validate: Invalid position input for {}, pass at most one of p, (lat, lon, alt), gps", F.name);
            throw std::invalid_argument(err_msg);
        }

        // Only one orientation representation at a time
        if ((has_C && has_q) || (has_C && has_eul) || (has_q && has_eul)) {
            std::string err_msg = std::format("vehicles::_StepOptions::validate: Invalid orientation input for {}, pass at most one of C, q, eul", F.name);
            throw std::invalid_argument(err_msg);
        }

        // Only one attitude-rate representation at a time
        if ((has_C_dot && (has_q_dot || has_w || has_eul_dot || has_wq)) ||
            (has_q_dot && (has_w || has_eul_dot || has_wq)) ||
            (has_w && (has_eul_dot || has_wq)) ||
            (has_eul_dot && has_wq)) {
            std::string err_msg = std::format("vehicles::_StepOptions::validate: Invalid attitude-rate input for {}, pass at most one of C_dot, q_dot, w, eul_dot, wq", F.name);
            throw std::invalid_argument(err_msg);
        }

        // RigidBodyState cannot be combined with any other state representations
        if (has_rbs && (
            has_H || has_C || has_p || has_q || has_eul ||
            has_C_dot || has_q_dot || has_w || has_eul_dot || has_wq ||
            has_v ||
            has_geo_any ||
            has_gps)) {
            std::string err_msg = std::format("vehicles::_StepOptions::validate: Invalid state input for {}, cannot pass H, C, p, q, eul, (lat, lon, alt), gps, C_dot, q_dot, w, eul_dot, wq with rbs", F.name);
            throw std::invalid_argument(err_msg);
        }

        // Only one aerodynamics representation at a time
        if (has_ads && (has_alpha || has_beta)) {
            std::string err_msg = std::format("vehicles::_StepOptions::validate: Invalid aerodynamics input for {}, cannot pass alpha, beta with ads", F.name);
            throw std::invalid_argument(err_msg);
        }
    }

    void Aircraft::step(const StepOptions& opts) {

        StepOptions::validate(opts);

        if (opts.NEDFrameECEFStepOpts.has_value()) {
            _StepOptions _opts; 
            _opts.lat = opts.NEDFrameECEFStepOpts->lat_NE;
            _opts.lon = opts.NEDFrameECEFStepOpts->lon_NE;
            _opts.alt = opts.NEDFrameECEFStepOpts->alt_NE;
            _opts.gps = opts.NEDFrameECEFStepOpts->gps_NE;

            _StepOptions::validate(NEDFrameECEF, _opts);
            Aircraft::step(NEDFrameECEF, _opts);
            Aircraft::step_dependents(NEDFrameECEF);
        }

        if (opts.FRDFrameECEFStepOpts.has_value()) {
            _StepOptions _opts;
            _opts.H = opts.FRDFrameECEFStepOpts->HEB;
            _opts.C = opts.FRDFrameECEFStepOpts->CEB;
            _opts.p = opts.FRDFrameECEFStepOpts->pE_BE;
            _opts.q = opts.FRDFrameECEFStepOpts->qEB;
            _opts.eul = opts.FRDFrameECEFStepOpts->eulEB;
            _opts.C_dot = opts.FRDFrameECEFStepOpts->CEB_dot;
            _opts.q_dot = opts.FRDFrameECEFStepOpts->qEB_dot;
            _opts.w = opts.FRDFrameECEFStepOpts->wB_BE;
            _opts.eul_dot = opts.FRDFrameECEFStepOpts->eulEB_dot;
            _opts.wq = opts.FRDFrameECEFStepOpts->wq_BE;
            _opts.v = opts.FRDFrameECEFStepOpts->vB_BE;
            _opts.lat = opts.FRDFrameECEFStepOpts->lat_BE;
            _opts.lon = opts.FRDFrameECEFStepOpts->lon_BE;
            _opts.alt = opts.FRDFrameECEFStepOpts->alt_BE;
            _opts.rbs = opts.FRDFrameECEFStepOpts->rbs_BE;
            _opts.gps = opts.FRDFrameECEFStepOpts->gps_BE;

            _StepOptions::validate(FRDFrameECEF, _opts);
            Aircraft::step(FRDFrameECEF, _opts);
            Aircraft::step_dependents(FRDFrameECEF);
        }

        if (opts.FRDFrameNEDStepOpts.has_value()) {
            _StepOptions _opts;
            _opts.H = opts.FRDFrameNEDStepOpts->HNB;
            _opts.C = opts.FRDFrameNEDStepOpts->CNB;
            _opts.p = opts.FRDFrameNEDStepOpts->pN_BN;
            _opts.q = opts.FRDFrameNEDStepOpts->qNB;
            _opts.eul = opts.FRDFrameNEDStepOpts->eulNB;
            _opts.C_dot = opts.FRDFrameNEDStepOpts->CNB_dot;
            _opts.q_dot = opts.FRDFrameNEDStepOpts->qNB_dot;
            _opts.w = opts.FRDFrameNEDStepOpts->wB_BN;
            _opts.eul_dot = opts.FRDFrameNEDStepOpts->eulNB_dot;
            _opts.wq = opts.FRDFrameNEDStepOpts->wq_BN;
            _opts.v = opts.FRDFrameNEDStepOpts->vB_BN;
            _opts.rbs = opts.FRDFrameNEDStepOpts->rbs_BN;

            _StepOptions::validate(FRDFrameNED, _opts);
            Aircraft::step(FRDFrameNED, _opts);
            Aircraft::step_dependents(FRDFrameNED);
        }

        if (opts.STABFrameFRDStepOpts.has_value()) {
            _StepOptions _opts;
            _opts.alpha = opts.STABFrameFRDStepOpts->alpha;
            _opts.ads = opts.STABFrameFRDStepOpts->ads;

            _StepOptions::validate(STABFrameFRD, _opts);
            Aircraft::step(STABFrameFRD, _opts);
            Aircraft::step_dependents(STABFrameFRD);
        }

        if (opts.WINDFrameSTABStepOpts.has_value()) {
            _StepOptions _opts; 
            _opts.beta = opts.WINDFrameSTABStepOpts->beta;
            _opts.ads = opts.WINDFrameSTABStepOpts->ads;

            _StepOptions::validate(WINDFrameSTAB, _opts);
            Aircraft::step(WINDFrameSTAB, _opts);
            Aircraft::step_dependents(WINDFrameSTAB);
        }

        // Sync all gravity vectors
        step_gravity();
    }



    void Aircraft::step(frames::NEDFrameECEF& F, const _StepOptions& opts) {

        bool recursive = !opts;
        frames::SetOptions NEDFrameECEFSetOpts;

        if (!recursive) {
            // final guard
            if (
                opts.H.has_value()          ||
                opts.C.has_value()          ||
                opts.p.has_value()          ||
                opts.q.has_value()          ||
                opts.eul.has_value()        ||
                opts.C_dot.has_value()      ||
                opts.q_dot.has_value()      ||
                opts.w.has_value()          ||
                opts.eul_dot.has_value()    ||
                opts.wq.has_value()         ||
                opts.v.has_value()          ||
                opts.g.has_value()
            ) { 
                std::string err_msg = "vehicles::Aircraft::_step: Invalid _StepOptions input passed for NEDFrameECEF";
                throw std::invalid_argument(err_msg);
            }

            if (opts.lat.has_value() && opts.lon.has_value() && opts.alt.has_value()) { 
                NEDFrameECEFSetOpts.p = geography::pE_from_lat_lon_alt(geography::GeographicState{ *opts.lat, *opts.lon, *opts.alt });
                NEDFrameECEFSetOpts.C = geography::CEN_from_lat_lon(*opts.lat, *opts.lon);
            }

            if (opts.gps.has_value()) {
                NEDFrameECEFSetOpts.p = geography::pE_from_lat_lon_alt(*opts.gps);
                NEDFrameECEFSetOpts.C = geography::CEN_from_lat_lon(opts.gps->lat, opts.gps->lon);
            }
        }

        else {
            std::string err_msg = "vehicles::Aircraft::_step: NEDFrameECEF is not a dependent to any step-able frame. It can never be recursively stepped. Something has gone wrong.";
            throw std::invalid_argument(err_msg);
        }

        // Defaults
        NEDFrameECEFSetOpts.C_dot = dynamics::OrientationMatrixRate{ constants::Zero3x3 };
        NEDFrameECEFSetOpts.v = dynamics::TranslationalVelocity{ constants::Zero3 };

        F.set(NEDFrameECEFSetOpts); 
    }

    void Aircraft::step(frames::FRDFrameECEF& F, const _StepOptions& opts) {

        bool recursive = !opts;
        frames::SetOptions FRDFrameECEFSetOpts;

        if (!recursive) {

            // final guard
            if (opts.g.has_value()) { 
                std::string err_msg = "vehicles::Aircraft::_step: Invalid _StepOptions input passed for FRDFrameECEF";
                throw std::invalid_argument(err_msg);
            }

            if (opts.H.has_value()) { 
                FRDFrameECEFSetOpts.H = *opts.H;
            }

            if (opts.C.has_value()) { 
                FRDFrameECEFSetOpts.C = *opts.C;
            }

            if (opts.p.has_value()) { 
                FRDFrameECEFSetOpts.p = *opts.p;
            }

            if (opts.q.has_value()) { 
                FRDFrameECEFSetOpts.q = *opts.q;
            }

            if (opts.eul.has_value()) { 
                FRDFrameECEFSetOpts.eul = *opts.eul;
            }

            if (opts.C_dot.has_value()) { 
                FRDFrameECEFSetOpts.C_dot = *opts.C_dot;
            }

            if (opts.q_dot.has_value()) { 
                FRDFrameECEFSetOpts.q_dot = *opts.q_dot;
            }

            if (opts.w.has_value()) { 
                FRDFrameECEFSetOpts.w = *opts.w;
            }

            if (opts.eul_dot.has_value()) { 
                FRDFrameECEFSetOpts.eul_dot = *opts.eul_dot;
            }

            if (opts.wq.has_value()) { 
                FRDFrameECEFSetOpts.wq = *opts.wq;
            }

            if (opts.v.has_value()) { 
                FRDFrameECEFSetOpts.v = *opts.v;
            }

            if (opts.lat.has_value() && opts.lon.has_value() && opts.alt.has_value()) { 
                dynamics::Position p = geography::pE_from_lat_lon_alt(geography::GeographicState{ *opts.lat, *opts.lon, *opts.alt });
                FRDFrameECEFSetOpts.p = p;
            }

            if (opts.rbs.has_value()) {
                FRDFrameECEFSetOpts.p = opts.rbs->p;
                FRDFrameECEFSetOpts.q = opts.rbs->q;
                FRDFrameECEFSetOpts.v = opts.rbs->v;
                FRDFrameECEFSetOpts.w = opts.rbs->w;
            }

            if (opts.gps.has_value()) {
                dynamics::Position p = geography::pE_from_lat_lon_alt(*opts.gps);
                FRDFrameECEFSetOpts.p = p;
            }
        }

        else {
            dynamics::Position p{ frames::transform_point(FRDFrameNED.HNB.p().data, NEDFrameECEF, frames::ECEF) };
            FRDFrameECEFSetOpts.p = p;
            dynamics::OrientationMatrix C{ FRDFrameNED.HNB.C().data * NEDFrameECEF.HEN.C().data };
            FRDFrameECEFSetOpts.C = C;
            FRDFrameECEFSetOpts.w = FRDFrameNED.wB_BN;
            FRDFrameECEFSetOpts.v = FRDFrameNED.vB_BN;
        }

        F.set(FRDFrameECEFSetOpts); 
    }

    void Aircraft::step(frames::FRDFrameNED& F, const _StepOptions& opts) {

        bool recursive = !opts;
        frames::SetOptions FRDFrameNEDSetOpts;

        if (!recursive) {

            // final guard
            if (opts.g.has_value()) { 
                std::string err_msg = "vehicles::Aircraft::_step: Invalid _StepOptions input passed for FRDFrameNED";
                throw std::invalid_argument(err_msg);
            }

            if (opts.H.has_value()) { 
                FRDFrameNEDSetOpts.H = *opts.H;
            }

            if (opts.C.has_value()) { 
                FRDFrameNEDSetOpts.C = *opts.C;
            }

            if (opts.p.has_value()) { 
                FRDFrameNEDSetOpts.p = *opts.p;
            }

            if (opts.q.has_value()) { 
                FRDFrameNEDSetOpts.q = *opts.q;
            }

            if (opts.eul.has_value()) { 
                FRDFrameNEDSetOpts.eul = *opts.eul;
            }

            if (opts.C_dot.has_value()) { 
                FRDFrameNEDSetOpts.C_dot = *opts.C_dot;
            }

            if (opts.q_dot.has_value()) { 
                FRDFrameNEDSetOpts.q_dot = *opts.q_dot;
            }

            if (opts.w.has_value()) { 
                FRDFrameNEDSetOpts.w = *opts.w;
            }

            if (opts.eul_dot.has_value()) { 
                FRDFrameNEDSetOpts.eul_dot = *opts.eul_dot;
            }

            if (opts.wq.has_value()) { 
                FRDFrameNEDSetOpts.wq = *opts.wq;
            }

            if (opts.v.has_value()) { 
                FRDFrameNEDSetOpts.v = *opts.v;
            }

            if (opts.rbs.has_value()) {
                FRDFrameNEDSetOpts.p = opts.rbs->p;
                FRDFrameNEDSetOpts.q = opts.rbs->q;
                FRDFrameNEDSetOpts.v = opts.rbs->v;
                FRDFrameNEDSetOpts.w = opts.rbs->w;
            }
        }

        else {
            FRDFrameNEDSetOpts.p = dynamics::Position{ frames::transform_point(FRDFrameECEF.HEB.p().data, frames::ECEF, NEDFrameECEF) };
            dynamics::OrientationMatrix C{ FRDFrameECEF.HEB.C().data *  NEDFrameECEF.HEN.C().data.transpose() };
            FRDFrameNEDSetOpts.C = C;
            FRDFrameNEDSetOpts.w = FRDFrameECEF.wB_BE;
            FRDFrameNEDSetOpts.v = FRDFrameECEF.vB_BE;
        }

        F.set(FRDFrameNEDSetOpts); 
    }

    void Aircraft::step(frames::STABFrameFRD& F, const _StepOptions& opts) {

        bool recursive = !opts;
        frames::SetOptions STABFrameFRDSetOpts;

        if (!recursive) {

            // final guard
            if (
                opts.H.has_value()           ||
                opts.C.has_value()           ||
                opts.p.has_value()           ||
                opts.q.has_value()           ||
                opts.eul.has_value()         ||
                opts.C_dot.has_value()       ||
                opts.q_dot.has_value()       ||
                opts.w.has_value()           ||
                opts.eul_dot.has_value()     ||
                opts.wq.has_value()          ||
                opts.v.has_value()           ||
                opts.g.has_value()
            ) { 
                std::string err_msg = "vehicles::Aircraft::_step: Invalid _StepOptions input passed for STABFrameFRD";
                throw std::invalid_argument(err_msg);
            }

            if (opts.alpha.has_value()) { 
                dynamics::OrientationMatrix C = aerodynamics::CBS(*opts.alpha);
                STABFrameFRDSetOpts.C = C;
            }

            if (opts.ads.has_value()) {
                dynamics::OrientationMatrix C = aerodynamics::CBS(opts.ads->alpha);
                STABFrameFRDSetOpts.C = C;
            }
        }

        else {
        }

        // Defaults
        STABFrameFRDSetOpts.p = dynamics::Position{ constants::Zero3 };
        STABFrameFRDSetOpts.C_dot = dynamics::OrientationMatrixRate{ constants::Zero3x3 };
        STABFrameFRDSetOpts.v = dynamics::TranslationalVelocity{ constants::Zero3 };

        F.set(STABFrameFRDSetOpts); 
    }

    void Aircraft::step(frames::WINDFrameSTAB& F, const _StepOptions& opts) {

        bool recursive = !opts;
        frames::SetOptions WINDFrameSTABSetOpts;

        if (!recursive) {

            // final guard
            if (
                opts.H.has_value()           ||
                opts.C.has_value()           ||
                opts.p.has_value()           ||
                opts.q.has_value()           ||
                opts.eul.has_value()         ||
                opts.C_dot.has_value()       ||
                opts.q_dot.has_value()       ||
                opts.w.has_value()           ||
                opts.eul_dot.has_value()     ||
                opts.wq.has_value()          ||
                opts.v.has_value()           ||
                opts.g.has_value()     
            ) { 
                std::string err_msg = "vehicles::Aircraft::_step: Invalid _StepOptions input passed for WINDFrameSTAB";
                throw std::invalid_argument(err_msg);
            }

            if (opts.beta.has_value()) { 
              dynamics::OrientationMatrix C = aerodynamics::CSW(*opts.beta);
                WINDFrameSTABSetOpts.C = C;
            }

            if (opts.ads.has_value()) {
                dynamics::OrientationMatrix C = aerodynamics::CSW(opts.ads->beta);
                WINDFrameSTABSetOpts.C = C;
            }
        }

        else {
        }

        // Defaults
        WINDFrameSTABSetOpts.p = dynamics::Position{ constants::Zero3 };
        WINDFrameSTABSetOpts.C_dot = dynamics::OrientationMatrixRate{ constants::Zero3x3 };
        WINDFrameSTABSetOpts.v = dynamics::TranslationalVelocity{ constants::Zero3 };

        F.set(WINDFrameSTABSetOpts); 
    }


    void Aircraft::step_dependents(frames::Frame& root) {
        std::queue<frames::Frame*> q;
        std::unordered_set<frames::Frame*> visited;

        q.push(&root);
        visited.insert(&root);

        while (!q.empty()) {
            frames::Frame* curr = q.front();
            q.pop();

            if (curr != &root) {
                step_dependent(*curr);
            }

            for (frames::Frame* dep : curr->dependents) {
                if (visited.insert(dep).second) q.push(dep); // if a dependent has not already been visited, mark it as visited and add it to the queue
            }
        }
    }

    void Aircraft::step_dependent(frames::Frame& F) {
        if (F.name == "NEDFrameECEF")   return step(static_cast<frames::NEDFrameECEF&>(F), {});
        if (F.name == "FRDFrameECEF")   return step(static_cast<frames::FRDFrameECEF&>(F), {});
        if (F.name == "FRDFrameNED")    return step(static_cast<frames::FRDFrameNED&>(F), {});
        if (F.name == "STABFrameFRD")   return step(static_cast<frames::STABFrameFRD&>(F), {});
        if (F.name == "WINDFrameSTAB")  return step(static_cast<frames::WINDFrameSTAB&>(F), {});

        std::string err_msg = "vehicles::Aircraft::step_dependent: Attempting to recursively step an unknown frame type";
        throw std::invalid_argument(err_msg);

    }

    _StepOptions::operator bool() const {
        return
            H.has_value()       ||
            C.has_value()       ||
            p.has_value()       ||
            q.has_value()       ||
            eul.has_value()     ||
            C_dot.has_value()   ||
            q_dot.has_value()   ||
            w.has_value()       ||
            eul_dot.has_value() ||
            wq.has_value()      ||
            v.has_value()       ||
            g.has_value()       ||
            lat.has_value()     ||
            lon.has_value()     ||
            alt.has_value()     ||
            alpha.has_value()   ||
            beta.has_value()    ||
            rbs.has_value()     ||
            gps.has_value()     ||
            ads.has_value();
    }


    void Aircraft::step_gravity() {

        // Sync NEDFrameECEF gravity
        NEDFrameECEF.gN = geography::gN();

        // Sync FRDFrameECEF gravity
        FRDFrameECEF.gB = geography::gB(FRDFrameECEF.HEB.p(), FRDFrameECEF.HEB.C());

        // Sync FRDFrameNED gravity
        // Two options for setting FRDFrameNED.gB
        // Option A: FRDFrameNED.gB = FRDFrameECEF.gB
        // Ensures gravity in the FRDFrameNED is always synced with the true ECEF‑based gravity vector at the aircraft's location
        // Option B: FRDFrameNED.gB = geography::gB(FRDFrameNED.HNB.C())
        // Performs a coordinate transform into the body frame of NEDFrameECEF's gravity vector
        // Because NEDFrameECEF is fixed at the initialization point and is never updated, this approximation slowly drifts from the true gravity vector (FRDFrameECEF.gB) as the aircraft moves away from the NED origin
        // Thus
        // Option A: best for global consistency of FRDFrameNED.gB with FRDFrameECEF.gB. Both frames agree on the same gravity vector derived from the true ECEF position
        // Option B: best for local congruence of FRDFrameNED with the fixed NEDFrameECEF frame, but FRDFrameNED.gB will diverge from the true ECEF‑based gravity vector (FRDFrameECEF.gB) as the aircraft moves
        // Verdict: For maximum calculation consistency, we choose option A
        FRDFrameNED.gB = FRDFrameECEF.gB;
        // FRDFrameNED.gB = geography::gB(FRDFrameNED.HNB.C());

        // Sync STABFrameFRD and WINDFrameSTAB gravity
        STABFrameFRD.gS = geography::gS(FRDFrameNED.gB, STABFrameFRD.HBS.C());
        WINDFrameSTAB.gW = geography::gW(STABFrameFRD.gS, WINDFrameSTAB.HSW.C());
    }

    void Aircraft::init_frames() {

        // Set default values
        frames::SetOptions initStepOptions;
        initStepOptions.H = dynamics::HomogeneousTransformationMatrix{ constants::HI };
        initStepOptions.w = dynamics::AngularVelocity{ constants::Zero3 };
        initStepOptions.v = dynamics::TranslationalVelocity{ constants::Zero3 };
        initStepOptions.g = dynamics::Gravity{ constants::Zero3 };

        NEDFrameECEF.set(initStepOptions);
        FRDFrameECEF.set(initStepOptions);
        FRDFrameNED.set(initStepOptions);
        STABFrameFRD.set(initStepOptions);
        WINDFrameSTAB.set(initStepOptions);

        // Set dependents, {X} -depends-on-> {Y}
        NEDFrameECEF.add_as_direct_dependent(&FRDFrameNED);    // {FRDFrameNED} -> {NEDFrameECEF}
    
        FRDFrameECEF.add_as_direct_dependent(&FRDFrameNED);    // {FRDFrameNED} -> {FRDFrameECEF}

        FRDFrameNED.add_as_direct_dependent(&FRDFrameECEF);    // {FRDFrameECEF} -> {FRDFrameNED}
        FRDFrameNED.add_as_direct_dependent(&STABFrameFRD);    // {STABFrameFRD} -> {FRDFrameNED}

        STABFrameFRD.add_as_direct_dependent(&WINDFrameSTAB);  // {WINDFrameSTAB} -> {STABFrameFRD}

    }

    Aircraft::Aircraft(
        const std::string& id,
        const structural::StructuralProperties& structural_properties,
        const aerodynamics::AerodynamicProperties& aerodynamic_properties,
        const actuators::ActuatorProperties& actuator_properties,
        const control::ControlProperties& control_properties,
        const avionics::AvionicsProperties& avionics_properties,
        const runtime::RuntimeProperties& runtime_properties,
        const guidance::GuidanceProperties& guidance_properties,
        const estimation::EstimationProperties& estimation_properties
    ) :
        id(id),
        NEDFrameECEF{},
        FRDFrameECEF{},
        FRDFrameNED{&NEDFrameECEF},
        STABFrameFRD{&FRDFrameNED},
        WINDFrameSTAB{&STABFrameFRD},
        structural_properties(structural_properties),
        aerodynamic_properties(aerodynamic_properties),
        actuator_properties(actuator_properties),
        control_properties(control_properties),
        avionics_properties(avionics_properties),
        runtime_properties(runtime_properties),
        guidance_properties(guidance_properties),
        estimation_properties(estimation_properties)
    {
        init_frames();
    }


    void Aircraft::print_state(int t, const atmospheric::Wind& windB) {
        const dynamics::RigidBodyState rbs = dynamics::compute_rigid_body_state(FRDFrameNED);
        const geography::GeographicState gps = geography::compute_geographic_state(FRDFrameECEF);
        const aerodynamics::AerodynamicState ads = aerodynamics::aerodynamic_state(FRDFrameNED, windB);

        const Eigen::Vector3d& p = rbs.p.data;
        const dynamics::EulerAngles& eul = FRDFrameNED.eulNB;
        const Eigen::Vector3d& v = rbs.v.data;
        const Eigen::Vector3d& w = rbs.w.data;
        const Eigen::Vector3d& g = FRDFrameNED.gB.data;
        const Eigen::Vector3d& wind = windB.data;

        std::cout
            << "t: " << t * constants::dt << " [s]" << "\n\n"
            << "p: " << p.x() << ", " << p.y() << ", " << p.z() << " [m]" << "\n\n"
            << "eul: " << util::rad_to_deg(eul.psi()) << ", " << util::rad_to_deg(eul.theta()) << ", " << util::rad_to_deg(eul.phi()) << " [deg]" << "\n\n"
            << "v: " << v.x() << ", " << v.y() << ", " << v.z() << " [ms^-1]" << "\n\n"
            << "w: " << util::rad_to_deg(w.x()) << ", " << util::rad_to_deg(w.y()) << ", " << util::rad_to_deg(w.z()) << " [deg/s]" << "\n\n"
            << "g: " << g.x() << ", " << g.y() << ", " << g.z() << " [ms^-2]" << "\n\n"
            << "lat: " << util::rad_to_deg(gps.lat.data) << ", lon: " << util::rad_to_deg(gps.lon.data) << " [deg]" << ", alt: " << gps.alt.data << " [m]" << "\n\n"
            << "alpha: " <<  util::rad_to_deg(ads.alpha.data) << ", beta: " <<  util::rad_to_deg(ads.beta.data) << " [deg]" << "\n\n"
            << "wind: " << wind.x() << ", " << wind.y() << ", " << wind.z() << " [m/s]" << "\n\n"
            << "-------------------------------------------------------------------------------" << "\n\n";
    }

}
