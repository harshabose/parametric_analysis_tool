#ifndef parametricAnalysis_h
#define parametricAnalysis_h

#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <string>
#include <cstring>
#include <ranges>

#include "nlohmann/json.hpp"

class parametric {
public:

    float MTOW, altitudeCruise, aspectRatio, oswaldRatio, velocityCruise, velocityClimb, velocityClimbFirstSegment, velocityClimbSecondSegment, altitudeClimb, altitudeClimbFirstSegment, altitudeClimbSecondSegment, CdoTarget;
    std::string resultType, XLSPath = "../outputs/parametrics.csv";
    float parametricPoint[2] = {0.0f, 0.0f};   //  {T_W, W_S}
    float surfaceArea = 0.0f, CLMAX_STALL_LIMIT{};
    
    parametric(
        float in_MTOW,
        float In_altitudeCruise, 
        float In_aspectRatio, 
        float In_oswaldRatio, 
        float In_velocityCruise, 
        float In_velocityClimb, 
        float In_climbRate,
        float In_velocityClimbFirstSegment,
        float In_climbRateFirstSegment,
        float In_velocityClimbSecondSegment,
        float In_climbRateSecondSegment,
        float In_altitudeClimbFirstSegment,
        float In_altitudeClimbSecondSegment,
        float In_altitudeClimb, 
        float In_CdoTarget,
        const bool in_read_from_jsons = false) {

        this->MTOW = in_MTOW;
        this->altitudeCruise = In_altitudeCruise;
        this->aspectRatio = In_aspectRatio;
        this->oswaldRatio = In_oswaldRatio;
        this->velocityCruise = In_velocityCruise;
        this->velocityClimb = In_velocityClimb;
        this->climbRate = In_climbRate;
        this->velocityClimbFirstSegment = In_velocityClimbFirstSegment;
        this->velocityClimbSecondSegment = In_velocityClimbSecondSegment;
        this->climbRateFirstSegment = In_climbRateFirstSegment;
        this->climbRateSecondSegment =  In_climbRateSecondSegment;
        this->altitudeClimb = In_altitudeClimb;
        this->altitudeClimbFirstSegment = In_altitudeClimbFirstSegment;
        this->altitudeClimbSecondSegment = In_altitudeClimbSecondSegment;
        this->CdoTarget = In_CdoTarget;

        try {
            this->T_W_Cruise = new float[this->numberOfPoints];
            this->T_W_Climb = new float[this->numberOfPoints];
            this->W_S = new float[this->numberOfPoints];
            this->T_W_ClimbFirstSegment = new float[this->numberOfPoints];
            this->T_W_ClimbSecondSegment = new float[this->numberOfPoints];
            this->W_S_minDrag = new float[this->numberOfPoints];
            this->T_W_maxThrust = new float[this->numberOfPoints];
            this->T_W_minDrag = new float[this->numberOfPoints];

            for (size_t i = 0; i < this->numberOfPoints; i++) {
                this->W_S[i] = 0.1f + (5000.0f - 0.1f) * static_cast<float>(i) / static_cast<float>(this->numberOfPoints - 1);
                this->T_W_minDrag[i] = 0.0f + (3.0f - 0.0f) * static_cast<float>(i) / static_cast<float>(this->numberOfPoints - 1);
            }
        } catch (const std::bad_alloc& e) {
            delete[] this->T_W_Cruise;
            delete[] this->T_W_Climb;
            delete[] this->T_W_ClimbFirstSegment;
            delete[] this->T_W_ClimbSecondSegment;
            delete[] this->W_S;
            delete[] this->W_S_minDrag;
            delete[] this->T_W_maxThrust;
            delete[] this->T_W_minDrag;

            std::cerr << "T_W vars not allocated properly" << e.what() << std::endl;
            throw;
        }

        this->WTOW = this->g * this->MTOW;
        this->densityCruise = getDensity(this->altitudeCruise);
        this->densityClimb = getDensity(this->altitudeClimb);
        this->densityClimbFirstSegment = getDensity(this->altitudeClimbFirstSegment);
        this->densityClimbSecondSegment = getDensity(this->altitudeClimbSecondSegment);
        this->inducedDragConstant = 1.0f / (this->pi * this->aspectRatio * this->oswaldRatio);

        if (in_read_from_jsons) this->read_results_from_json();
    }


    ~parametric() {
        this->clearMallocMemory();
        std::cout << "'Parametrics' memory cleared!" << std::endl;
    }

    parametric(const parametric& newMoveFromParametric): CLMAX_STALL_LIMIT(0), climbRateFirstSegment(0),
                                                         climbRateSecondSegment(0),
                                                         climbRate(0) {
        //object create copy
        this->MTOW = newMoveFromParametric.MTOW;
        this->altitudeCruise = newMoveFromParametric.altitudeCruise;
        this->aspectRatio = newMoveFromParametric.aspectRatio;
        this->oswaldRatio = newMoveFromParametric.oswaldRatio;
        this->velocityCruise = newMoveFromParametric.velocityCruise;
        this->velocityClimb = newMoveFromParametric.velocityClimb;
        // this->climbRate = newMoveFromParametric.climbRate;
        this->velocityClimbFirstSegment = newMoveFromParametric.velocityClimbFirstSegment;
        this->velocityClimbSecondSegment = newMoveFromParametric.velocityClimbSecondSegment;
        // this->climbRateFirstSegment = newMoveFromParametric.climbRateFirstSegment;
        // this->climbRateSecondSegment = newMoveFromParametric.climbRateSecondSegment;
        this->altitudeClimb = newMoveFromParametric.altitudeClimb;
        this->altitudeClimbFirstSegment = newMoveFromParametric.altitudeClimbFirstSegment;
        this->altitudeClimbSecondSegment = newMoveFromParametric.altitudeClimbSecondSegment;
        this->CdoTarget = newMoveFromParametric.CdoTarget;
        this->resultType = newMoveFromParametric.resultType;
        this->XLSPath = newMoveFromParametric.XLSPath;
        memcpy(this->parametricPoint, newMoveFromParametric.parametricPoint,
               sizeof(newMoveFromParametric.parametricPoint));

        this->WTOW = newMoveFromParametric.WTOW;
        this->densityCruise = newMoveFromParametric.densityCruise;
        this->densityClimb = newMoveFromParametric.densityClimb;
        this->densityClimbFirstSegment = newMoveFromParametric.densityClimbFirstSegment;
        this->densityClimbSecondSegment = newMoveFromParametric.densityClimbSecondSegment;
        this->inducedDragConstant = newMoveFromParametric.inducedDragConstant;

        try {
            this->T_W_Cruise = new float[this->numberOfPoints];
            this->T_W_Climb = new float[this->numberOfPoints];
            this->W_S = new float[this->numberOfPoints];
            this->T_W_ClimbFirstSegment = new float[this->numberOfPoints];
            this->T_W_ClimbSecondSegment = new float[this->numberOfPoints];
            this->W_S_minDrag = new float[this->numberOfPoints];
            this->T_W_maxThrust = new float[this->numberOfPoints];
            this->T_W_minDrag = new float[this->numberOfPoints];

            size_t sizeToCopy = sizeof(float) * this->numberOfPoints;

            memcpy(this->T_W_Cruise, newMoveFromParametric.T_W_Cruise, sizeToCopy);
            memcpy(this->T_W_Climb, newMoveFromParametric.T_W_Climb, sizeToCopy);
            memcpy(this->T_W_ClimbFirstSegment, newMoveFromParametric.T_W_ClimbFirstSegment, sizeToCopy);
            memcpy(this->T_W_ClimbSecondSegment, newMoveFromParametric.T_W_ClimbSecondSegment, sizeToCopy);
            memcpy(this->W_S, newMoveFromParametric.W_S, sizeToCopy);
            memcpy(this->W_S_minDrag, newMoveFromParametric.W_S_minDrag, sizeToCopy);
            memcpy(this->T_W_maxThrust, newMoveFromParametric.T_W_maxThrust, sizeToCopy);
            memcpy(this->T_W_minDrag, newMoveFromParametric.T_W_minDrag, sizeToCopy);
        } catch (const std::exception &e) {
            delete[] this->T_W_Cruise;
            delete[] this->T_W_Climb;
            delete[] this->T_W_ClimbFirstSegment;
            delete[] this->T_W_ClimbSecondSegment;
            delete[] this->W_S;
            delete[] this->W_S_minDrag;
            delete[] this->T_W_maxThrust;
            delete[] this->T_W_minDrag;

            std::cerr << "'Copying' T_W vars not allocated properly" << e.what() << std::endl;
            throw;
        }
    }

    parametric& operator=(const parametric& copyParametric) {   //object copy
        if (this != &copyParametric) {
            this->MTOW = copyParametric.MTOW;
            this->altitudeCruise = copyParametric.altitudeCruise;
            this->aspectRatio = copyParametric.aspectRatio;
            this->oswaldRatio = copyParametric.oswaldRatio;
            this->velocityCruise = copyParametric.velocityCruise;
            this->velocityClimb = copyParametric.velocityClimb;
            this->velocityClimbFirstSegment = copyParametric.velocityClimbFirstSegment;
            this->velocityClimbSecondSegment = copyParametric.velocityClimbSecondSegment;
            this->altitudeClimb = copyParametric.altitudeClimb;
            this->altitudeClimbFirstSegment = copyParametric.altitudeClimbFirstSegment;
            this->altitudeClimbSecondSegment = copyParametric.altitudeClimbSecondSegment;
            this->CdoTarget = copyParametric.CdoTarget;
            this->resultType = copyParametric.resultType;
            this->XLSPath = copyParametric.XLSPath;
            memcpy(this->parametricPoint, copyParametric.parametricPoint, sizeof(copyParametric.parametricPoint));

            this->WTOW = copyParametric.WTOW;
            this->densityCruise = copyParametric.densityCruise;
            this->densityClimb = copyParametric.densityClimb;
            this->densityClimbFirstSegment = copyParametric.densityClimbFirstSegment;
            this->densityClimbSecondSegment = copyParametric.densityClimbSecondSegment;
            this->inducedDragConstant = copyParametric.inducedDragConstant;

            size_t sizeToCopy = sizeof(float) * this->numberOfPoints;

            memcpy(this->T_W_Cruise, copyParametric.T_W_Cruise, sizeToCopy);
            memcpy(this->T_W_Climb, copyParametric.T_W_Climb, sizeToCopy);
            memcpy(this->T_W_ClimbFirstSegment, copyParametric.T_W_ClimbFirstSegment, sizeToCopy);
            memcpy(this->T_W_ClimbSecondSegment, copyParametric.T_W_ClimbSecondSegment, sizeToCopy);
            memcpy(this->W_S, copyParametric.W_S, sizeToCopy);
            memcpy(this->W_S_minDrag, copyParametric.W_S_minDrag, sizeToCopy);
            memcpy(this->T_W_maxThrust, copyParametric.T_W_maxThrust, sizeToCopy);
            memcpy(this->T_W_minDrag, copyParametric.T_W_minDrag, sizeToCopy);
        }

        return *this;
    }

    parametric(parametric&& newMoveFromParametric) noexcept { //object create move
        this->MTOW = newMoveFromParametric.MTOW;
        this->altitudeCruise = newMoveFromParametric.altitudeCruise;
        this->aspectRatio = newMoveFromParametric.aspectRatio;
        this->oswaldRatio = newMoveFromParametric.oswaldRatio;
        this->velocityCruise = newMoveFromParametric.velocityCruise;
        this->velocityClimb = newMoveFromParametric.velocityClimb;
        this->velocityClimbFirstSegment = newMoveFromParametric.velocityClimbFirstSegment;
        this->velocityClimbSecondSegment = newMoveFromParametric.velocityClimbSecondSegment;
        this->altitudeClimb = newMoveFromParametric.altitudeClimb;
        this->altitudeClimbFirstSegment = newMoveFromParametric.altitudeClimbFirstSegment;
        this->altitudeClimbSecondSegment = newMoveFromParametric.altitudeClimbSecondSegment;
        this->CdoTarget = newMoveFromParametric.CdoTarget;
        this->resultType = newMoveFromParametric.resultType;
        this->XLSPath = newMoveFromParametric.XLSPath;
        memcpy(this->parametricPoint, newMoveFromParametric.parametricPoint, sizeof(newMoveFromParametric.parametricPoint));

        this->WTOW = newMoveFromParametric.WTOW;
        this->densityCruise = newMoveFromParametric.densityCruise;
        this->densityClimb = newMoveFromParametric.densityClimb;
        this->densityClimbFirstSegment = newMoveFromParametric.densityClimbFirstSegment;
        this->densityClimbSecondSegment = newMoveFromParametric.densityClimbSecondSegment;
        this->inducedDragConstant = newMoveFromParametric.inducedDragConstant;

        this->T_W_Cruise = newMoveFromParametric.T_W_Cruise;
        this->T_W_Climb = newMoveFromParametric.T_W_Climb;
        this->T_W_ClimbFirstSegment = newMoveFromParametric.T_W_ClimbFirstSegment;
        this->T_W_ClimbSecondSegment = newMoveFromParametric.T_W_ClimbSecondSegment;
        this->W_S = newMoveFromParametric.W_S;
        this->W_S_minDrag = newMoveFromParametric.W_S_minDrag;
        this->T_W_maxThrust = newMoveFromParametric.T_W_maxThrust;
        this->T_W_minDrag = newMoveFromParametric.T_W_minDrag;


        newMoveFromParametric.MTOW = 0.0f;
        newMoveFromParametric.altitudeCruise = 0.0f;
        newMoveFromParametric.aspectRatio = 0.0f;
        newMoveFromParametric.oswaldRatio = 0.0f;
        newMoveFromParametric.velocityCruise = 0.0f;
        newMoveFromParametric.velocityClimb = 0.0f;
        newMoveFromParametric.velocityClimbFirstSegment = 0.0f;
        newMoveFromParametric.velocityClimbSecondSegment = 0.0f;
        newMoveFromParametric.altitudeClimb = 0.0f;
        newMoveFromParametric.altitudeClimbFirstSegment = 0.0f;
        newMoveFromParametric.altitudeClimbSecondSegment = 0.0f;
        newMoveFromParametric.CdoTarget = 0.0f;
        newMoveFromParametric.resultType = 0.0f;
        newMoveFromParametric.XLSPath.clear();
        newMoveFromParametric.parametricPoint[0] = 0.0f;
        newMoveFromParametric.parametricPoint[1] = 0.0f;

        newMoveFromParametric.WTOW = 0.0f;
        newMoveFromParametric.densityCruise = 0.0f;
        newMoveFromParametric.densityClimb = 0.0f;
        newMoveFromParametric.densityClimbFirstSegment = 0.0f;
        newMoveFromParametric.densityClimbSecondSegment = 0.0f;
        newMoveFromParametric.inducedDragConstant = 0.0f;

        newMoveFromParametric.T_W_Cruise = nullptr;
        newMoveFromParametric.T_W_Climb = nullptr;
        newMoveFromParametric.T_W_ClimbFirstSegment = nullptr;
        newMoveFromParametric.T_W_ClimbSecondSegment = nullptr;
        newMoveFromParametric.W_S = nullptr;
        newMoveFromParametric.W_S_minDrag = nullptr;
        newMoveFromParametric.T_W_maxThrust = nullptr;
        newMoveFromParametric.T_W_minDrag = nullptr;
    }

    parametric& operator=(parametric&& moveParametric) noexcept {   //object move
        if (this != &moveParametric) {
            this->MTOW = moveParametric.MTOW;
            this->altitudeCruise = moveParametric.altitudeCruise;
            this->aspectRatio = moveParametric.aspectRatio;
            this->oswaldRatio = moveParametric.oswaldRatio;
            this->velocityCruise = moveParametric.velocityCruise;
            this->velocityClimb = moveParametric.velocityClimb;
            this->velocityClimbFirstSegment = moveParametric.velocityClimbFirstSegment;
            this->velocityClimbSecondSegment = moveParametric.velocityClimbSecondSegment;
            this->altitudeClimb = moveParametric.altitudeClimb;
            this->altitudeClimbFirstSegment = moveParametric.altitudeClimbFirstSegment;
            this->altitudeClimbSecondSegment = moveParametric.altitudeClimbSecondSegment;
            this->CdoTarget = moveParametric.CdoTarget;
            this->resultType = moveParametric.resultType;
            this->XLSPath = moveParametric.XLSPath;
            memcpy(this->parametricPoint, moveParametric.parametricPoint, sizeof(moveParametric.parametricPoint));

            this->WTOW = moveParametric.WTOW;
            this->densityCruise = moveParametric.densityCruise;
            this->densityClimb = moveParametric.densityClimb;
            this->densityClimbFirstSegment = moveParametric.densityClimbFirstSegment;
            this->densityClimbSecondSegment = moveParametric.densityClimbSecondSegment;
            this->inducedDragConstant = moveParametric.inducedDragConstant;

            this->clearMallocMemory();
            std::cout << "Malloc memory of 'moveParametric' object have been cleared!" << std::endl;

            this->T_W_Cruise = moveParametric.T_W_Cruise;
            this->T_W_Climb = moveParametric.T_W_Climb;
            this->T_W_ClimbFirstSegment = moveParametric.T_W_ClimbFirstSegment;
            this->T_W_ClimbSecondSegment = moveParametric.T_W_ClimbSecondSegment;
            this->W_S = moveParametric.W_S;
            this->W_S_minDrag = moveParametric.W_S_minDrag;
            this->T_W_maxThrust = moveParametric.T_W_maxThrust;
            this->T_W_minDrag = moveParametric.T_W_minDrag;


            moveParametric.MTOW = 0.0f;
            moveParametric.altitudeCruise = 0.0f;
            moveParametric.aspectRatio = 0.0f;
            moveParametric.oswaldRatio = 0.0f;
            moveParametric.velocityCruise = 0.0f;
            moveParametric.velocityClimb = 0.0f;
            moveParametric.velocityClimbFirstSegment = 0.0f;
            moveParametric.velocityClimbSecondSegment = 0.0f;
            moveParametric.altitudeClimb = 0.0f;
            moveParametric.altitudeClimbFirstSegment = 0.0f;
            moveParametric.altitudeClimbSecondSegment = 0.0f;
            moveParametric.CdoTarget = 0.0f;
            moveParametric.resultType = 0.0f;
            moveParametric.XLSPath.clear();
            moveParametric.parametricPoint[0] = 0.0f;
            moveParametric.parametricPoint[1] = 0.0f;

            moveParametric.WTOW = 0.0f;
            moveParametric.densityCruise = 0.0f;
            moveParametric.densityClimb = 0.0f;
            moveParametric.densityClimbFirstSegment = 0.0f;
            moveParametric.densityClimbSecondSegment = 0.0f;
            moveParametric.inducedDragConstant = 0.0f;

            moveParametric.T_W_Cruise = nullptr;
            moveParametric.T_W_Climb = nullptr;
            moveParametric.T_W_ClimbFirstSegment = nullptr;
            moveParametric.T_W_ClimbSecondSegment = nullptr;
            moveParametric.W_S = nullptr;
            moveParametric.W_S_minDrag = nullptr;
            moveParametric.T_W_maxThrust = nullptr;
            moveParametric.T_W_minDrag = nullptr;
        }

        return *this;
    }



    float* getParametrics (const bool printToCSV = false) {
        this->getCruiseParamatrics();
        this->getVTOSSClimbparametrics(this->T_W_ClimbFirstSegment,
                                       this->densityClimbFirstSegment, 
                                       this->velocityClimbFirstSegment,  
                                       atanf(this->climbRateFirstSegment), 
                                       1.0f);

        this->getClimbParametrics(this->T_W_ClimbSecondSegment, 
                                  this->densityClimbSecondSegment, 
                                  this->velocityClimbSecondSegment, 
                                  this->climbRateSecondSegment);

        this->getClimbParametrics(this->T_W_Climb, this->densityClimb, this->velocityClimb, this->climbRate);

        this->getMinimumDragParametrics();
        this->getMaxThrustParametrics();

        this->getParametricPoint();
        if (printToCSV) this->printToXLS();
        this->surfaceArea = this->WTOW / this-> parametricPoint[1];
        this->find_max_CL_STALL_LIMIT();

        return this->parametricPoint;
    }

    float find_max_CL_STALL_LIMIT (const float stall_limit_velocity = 31.3889f) {
        const std::vector<float> check_altitudes = {this->altitudeClimbFirstSegment, this->altitudeClimbSecondSegment, this->altitudeClimb, this->altitudeCruise};
        std::vector<float> CLMAX_vector;

        for (std::size_t i = 0; i < check_altitudes.size(); i++) {
            const float dynamic_constant = 0.5f * getDensity(0.0f) * stall_limit_velocity * stall_limit_velocity;
            CLMAX_vector.push_back((1.0f / dynamic_constant) * this->parametricPoint[1]);
        }

        this->CLMAX_STALL_LIMIT = *std::max_element(CLMAX_vector.begin(), CLMAX_vector.end());

        return this->CLMAX_STALL_LIMIT;
    }

    void print_results_to_json (const std::string& path = "parametric_analysis_h.json", const std::string& run = "run_1/") {
        const std::string file_path = "../assets/" + run + path;
        std::ofstream parametric_file(file_path);
        const nlohmann::json json_data = this->convert_parametric_to_json();
        try {
            if(!parametric_file.is_open()) {
                throw std::runtime_error("Could not open file for writing: " + file_path);
            }
            std::cout << "Writing to " << path  << std::endl;
            parametric_file << json_data.dump(4);
            parametric_file.close();
        } catch (const std::runtime_error& e) {
            std::cerr << "An error occurred while writting to 'parametric' JSON: " << e.what() << std::endl;
            std::cerr << "Parametric JSON DATA NOT STORED" << std::endl;
        }
    }

    void read_results_from_json (const std::string& path = "parametric_analysis_h.json", const std::string& run = "run_1/") {
        const std::string file_path = "../assets/" + run + path;
        std::ifstream parametric_file(file_path, std::ios::ate);

        if (!parametric_file.is_open() && (parametric_file.tellg() != 0)) {
            throw std::runtime_error("Could not open file for reading or it is empty: " + file_path);
        }

        parametric_file.seekg(0, std::ios::beg);

        const nlohmann::json json_data = nlohmann::json::parse(parametric_file);
        parametric_file.close();

        try {
            this->convert_json_to_parametric(json_data);
        } catch (std::exception& e) {
            std::cerr << e.what() << std::endl;
            throw;
        }
    }

private:
    float WTOW, densityCruise, densityClimb, densityClimbFirstSegment, densityClimbSecondSegment;
    float inducedDragConstant, climbRateFirstSegment{}, climbRateSecondSegment{}, climbRate{};
    const unsigned int numberOfPoints = 500;
    float* T_W_Cruise = nullptr;
    float* T_W_ClimbSecondSegment = nullptr;
    float* T_W_ClimbFirstSegment = nullptr;
    float* T_W_Climb = nullptr;
    float* W_S = nullptr;
    float* W_S_minDrag = nullptr;
    float* T_W_maxThrust = nullptr;
    float* T_W_minDrag = nullptr;
    const float g = 9.80665f, pi = 3.14159265f;

    static float getDensity (const float altitude) {
        return(1.225f * pow((1.0f - 22.558e-6f * altitude), 4.2559f));
    }

    void getCruiseParamatrics() const {
        const float A = 0.5f * this->densityCruise * this->CdoTarget * pow(this->velocityCruise, 2.0f);
        const float B = 2.0f * this->inducedDragConstant / (this->densityCruise * pow(velocityCruise, 2.0f));

        for(size_t i = 0; i < this->numberOfPoints; i++) {
            this->T_W_Cruise[i] = (A / this->W_S[i]) + B * this->W_S[i];
        }
    }

    void getClimbParametrics(float* ClimbT_W,
                            const float density, 
                            const float velocity, 
                            const float speedRatio, 
                            const float accelerationRatio = 0.0f) const {
        for(size_t i = 0; i < this->numberOfPoints; i++) {
           ClimbT_W[i] = accelerationRatio + speedRatio +
                         (0.5f * density * pow(velocity, 2.0f) * this->CdoTarget / this->W_S[i]) +
                         (this->inducedDragConstant * this->W_S[i] / (0.5f * density * pow(velocity, 2.0f)));
        }
    }

    void getVTOSSClimbparametrics(float* ClimbT_W,
                                 const float density, 
                                 const float velocity, 
                                 const float alpha, 
                                 const float accelerationRatio = 0.0f) const {
        const float A = (0.5f * density * pow(velocity, 2.0f)) * this->CdoTarget;
        for(size_t i = 0; i < this->numberOfPoints; i++) {
            ClimbT_W[i] = sqrt (
                pow(A / this->W_S[i], 2.0f) +
                2.0f * (A / this->W_S[i]) * sin(alpha * this->pi / 180.0f) +
                accelerationRatio
            );
        }
    }

    void getMinimumDragParametrics() const {
        for (size_t i = 0; i < this->numberOfPoints; i++) {
             this->W_S_minDrag[i] = sqrt(this->CdoTarget / this->inducedDragConstant) *
                            (0.5f * this->densityCruise * pow(this->velocityCruise, 2.0f));
        }
    }

    void getMaxThrustParametrics() {
        for (size_t i = 0; i < this->numberOfPoints; i++) {
            this->T_W_maxThrust[i] = 1.5;
        }
    }

    void getParametricPoint() {
        float finalPoint[] = {0.0f, 0.0f};    //{T_W, W_S}
        float* combinedT_WPointers[] = {
            this->T_W_Cruise,
            this->T_W_Climb,
            this->T_W_ClimbFirstSegment,
            this->T_W_ClimbSecondSegment,
            this->T_W_minDrag,
            this->T_W_maxThrust
        };
        float* combinedW_SPointers[] = {
            this->W_S,
            this->W_S,
            this->W_S,
            this->W_S,
            this->W_S_minDrag,
            this->W_S
        };

        finalPoint[0] = combinedT_WPointers[5][0];
        float finalParametricW_S = combinedW_SPointers[0][this->findClosest(combinedT_WPointers[0], finalPoint[0])];
        for (size_t i = 0; i < 5; i++) {
            float currentParametricW_S = combinedW_SPointers[i][this->findClosest(combinedT_WPointers[i], finalPoint[0])];
            
            if (currentParametricW_S > finalParametricW_S) {
                finalParametricW_S = currentParametricW_S;
            }
        }

        finalPoint[1] = finalParametricW_S;

        memcpy(this->parametricPoint, finalPoint, sizeof(this->parametricPoint));
    }

    size_t findClosest(const float* destinationArray, const float targetValue) const {
        float closest = destinationArray[0];
        size_t index = 0;
        float minDifference = std::abs(targetValue - closest);

        for (size_t i = 0; i < this->numberOfPoints; i++) {
            float currentDifference = std::abs(targetValue - destinationArray[i]);

            if (currentDifference < minDifference) {
                closest = destinationArray[i];
                minDifference = currentDifference;
                index = i;
            }
        }

        return index;
    }

    void printToXLS() const {
        {
            std::ofstream file(this->XLSPath);
            if (!file.is_open()) {
                std::cerr << "Error: Unable to open or create 'parametric.csv'" << std::endl;
                return;
            }

            file<< "W/S" << "," << "T/W First Segment Climb" << ","
                << "T/W Second Segment Climb"  << ","
                << "T/W Final Segment Climb"  << ","
                << "T/W Cruise" << ","
                << "T/S max Thrust" << ","
                << "W/S min Drag" << ","
                << "T/W min Drag" << ",";
            file << "\n";

            for (size_t i = 0; i < this->numberOfPoints; i++) {
                file<< this->W_S[i] << ","
                    << this->T_W_ClimbFirstSegment[i] << ","
                    << this->T_W_ClimbSecondSegment[i]  << ","
                    << this->T_W_Climb[i]  << ","
                    << this->T_W_Cruise[i] << ","
                    << this->T_W_maxThrust[i] << ","
                    << this->W_S_minDrag[i] << ","
                    << this->T_W_minDrag[i] << ",";
                file << "\n";
            }

            file.close();
        }
    }

    void clearMallocMemory() const {
        delete[] this->T_W_Cruise;
        delete[] this->T_W_Climb;
        delete[] this->T_W_ClimbFirstSegment;
        delete[] this->T_W_ClimbSecondSegment;
        delete[] this->W_S;
        delete[] this->W_S_minDrag;
        delete[] this->T_W_maxThrust;
        delete[] this->T_W_minDrag;
    }

    nlohmann::json convert_parametric_to_json () {
        nlohmann::json j;

        //public
        j["MTOW"] = this->MTOW;
        j["altitudeCruise"] = this->altitudeCruise;
        j["aspectRatio"] = this->aspectRatio;
        j["oswaldRatio"] = this->oswaldRatio;
        j["velocityCruise"] = this->velocityCruise;
        j["velocityClimb"] = this->velocityClimb;
        j["velocityClimbFirstSegment"] = this->velocityClimbFirstSegment;
        j["velocityClimbSecondSegment"] = this->velocityClimbSecondSegment;
        j["altitudeClimb"] = this->altitudeClimb;
        j["altitudeClimbFirstSegment"] = this->altitudeClimbFirstSegment;
        j["altitudeClimbSecondSegment"] = this->altitudeClimbSecondSegment;
        j["CdoTarget"] = this->CdoTarget;
        j["resultType"] = this->resultType;
        j["XLSPath"] = this->XLSPath;
        j["parametricPoint"] = {this->parametricPoint[0], this->parametricPoint[1]};
        j["surfaceArea"] = this->surfaceArea;
        j["CLMAX_STALL_LIMIT"] = this->CLMAX_STALL_LIMIT;

        // private
        j["WTOW"] = this->WTOW;
        j["densityCruise"] = this->densityCruise;
        j["densityClimb"] = this->densityClimb;
        j["densityClimbFirstSegment"] = this->densityClimbFirstSegment;
        j["densityClimbSecondSegment"] = this->densityClimbSecondSegment;
        j["inducedDragConstant"] = this->inducedDragConstant;
        j["climbRateFirstSegment"] = this->climbRateFirstSegment;
        j["climbRateSecondSegment"] = this->climbRateSecondSegment;
        j["climbRate"] = this->climbRate;

        return j;
    }

    void convert_json_to_parametric (const nlohmann::json& j) {
        try {
            //public
            this->MTOW = j.at("MTOW").get<float>();
            this->altitudeCruise = j.at("altitudeCruise").get<float>();
            this->aspectRatio = j.at("aspectRatio").get<float>();
            this->oswaldRatio = j.at("oswaldRatio").get<float>();
            this->velocityCruise = j.at("velocityCruise").get<float>();
            this->velocityClimb = j.at("velocityClimb").get<float>();
            this->velocityClimbFirstSegment = j.at("velocityClimbFirstSegment").get<float>();
            this->velocityClimbSecondSegment = j.at("velocityClimbSecondSegment").get<float>();
            this->altitudeClimb = j.at("altitudeClimb").get<float>();
            this->altitudeClimbFirstSegment = j.at("altitudeClimbFirstSegment").get<float>();
            this->altitudeClimbSecondSegment = j.at("altitudeClimbSecondSegment").get<float>();
            this->CdoTarget = j.at("CdoTarget").get<float>();
            this->resultType = j.at("resultType").get<std::string>();
            this->XLSPath = j.at("XLSPath").get<std::string>();
            if (j.contains("parametricPoint") && j["parametricPoint"].is_array()) {
                auto parametricPointJson = j["parametricPoint"];
                if (parametricPointJson.size() >= 2) {
                    this->parametricPoint[0] = parametricPointJson.at(0).get<float>();
                    this->parametricPoint[1] = parametricPointJson.at(1).get<float>();
                } else throw nlohmann::json::out_of_range::create(404, "parametricPoint JSON array has size not equal to 2", nullptr);
            } else throw nlohmann::json::out_of_range::create(404, "parametricPoint JSON data has no key 'parametricPoint or JSON data is not an array", nullptr);
            this->surfaceArea = j.at("surfaceArea").get<float>();
            this->CLMAX_STALL_LIMIT = j.at("CLMAX_STALL_LIMIT").get<float>();

            //private
            this->WTOW = j.at("WTOW").get<float>();
            this->densityCruise = j.at("densityCruise").get<float>();
            this->densityClimb = j.at("densityClimb").get<float>();
            this->densityClimbFirstSegment = j.at("densityClimbFirstSegment").get<float>();
            this->densityClimbSecondSegment = j.at("densityClimbSecondSegment").get<float>();
            this->inducedDragConstant = j.at("inducedDragConstant").get<float>();
            this->climbRateFirstSegment = j.at("climbRateFirstSegment").get<float>();
            this->climbRateSecondSegment = j.at("climbRateSecondSegment").get<float>();
            this->climbRate = j.at("climbRate").get<float>();
        } catch (std::exception& e) {
            std::cerr << "Converting from JSON to Parametric Failed: " << e.what() << std::endl;
            throw;
        }
    }

};

std::shared_ptr<parametricAnalysis_h::parametric> parametric_analysis (bool read_from_jsons = false);    // header implementation


#endif