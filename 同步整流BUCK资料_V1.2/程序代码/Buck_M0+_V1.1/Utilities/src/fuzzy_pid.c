#include "fuzzy_pid.h"

const float eRules[7]={-3.0f,-2.0f,-1.0f,0.0f,1.0f,2.0f,3.0f};
const float ecRules[7]={-3.0f,-2.0f,-1.0f,0.0f,1.0f,2.0f,3.0f};
const float fuzzy_kpRule[4]={0.0f,2.0f,4.0f,6.0f}; //Kp的模糊子集
const float fuzzy_kiRule[4]={0.0f,0.01f,0.02f,0.04f}; //Ki的模糊子集
float eFuzzy[2]={0.0f,0.0f};
float ecFuzzy[2]={0.0f,0.0f};
int8_t pe=0, pec=0;
	
const int8_t kpRules[7][7]=
{
	3, 3, 3, 3, 3, 3, 3,
    2, 2, 2, 2, 1, 2, 2,
	1, 1, 1, 1, 1, 1, 1,
	1, 1, 0, 1, 0, 1, 1,
	0, 0, 1, 0, 0, 1, 0,
	0, 1, 0, 1, 0, 0, 2,
	3, 3, 3, 3, 3, 3, 3
};

const int8_t kiRules[7][7]=
{
	0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0,
	2, 0, 0, 0, 0, 0, 1,
	3, 3, 3, 3, 3, 3, 3
};

void FuzzyPID_Calc(float f_e, float f_ec, float* kp, float* ki)
{
	int8_t num=0;
	float Kp_calcu=0.0f;
	float kpFuzzy[4] = {0.0f,0.0f,0.0f,0.0f}; //隶属于Kp的隶属程度
	float kiFuzzy[4] = {0.0f,0.0f,0.0f,0.0f}; //隶属于Kp的隶属程度
	float e2Fuzzy[4] = {0.0f,0.0f,0.0f,0.0f};
	
	FuzzySubjection(f_e, f_ec);
	
	e2Fuzzy[0] = eFuzzy[0] * ecFuzzy[0];
	e2Fuzzy[1] = eFuzzy[0] * ecFuzzy[1];
	e2Fuzzy[2] = eFuzzy[1] * ecFuzzy[0];
	e2Fuzzy[3] = eFuzzy[1] * ecFuzzy[1];
	
	/*********查询Kp模糊规则表*********/
    num = kpRules[pe][pec];
    kpFuzzy[num] += e2Fuzzy[0];
    num = kpRules[pe][pec+1];
    kpFuzzy[num] += e2Fuzzy[1];
    num = kpRules[pe+1][pec];
    kpFuzzy[num] += e2Fuzzy[2];
    num = kpRules[pe+1][pec+1];
    kpFuzzy[num] += e2Fuzzy[3];
	
	/*********Ki查询模糊规则表*********/
    num = kiRules[pe][pec];
    kiFuzzy[num] += e2Fuzzy[0];
    num = kiRules[pe][pec+1];
    kiFuzzy[num] += e2Fuzzy[1];
    num = kiRules[pe+1][pec];
    kiFuzzy[num] += e2Fuzzy[2];
    num = kiRules[pe+1][pec+1];
    kiFuzzy[num] += e2Fuzzy[3];
	
    /*********Kp加权平均法解模糊*********/
    *kp = kpFuzzy[0] * fuzzy_kpRule[0] + kpFuzzy[1] * fuzzy_kpRule[1]\
			   + kpFuzzy[2] * fuzzy_kpRule[2] + kpFuzzy[3] * fuzzy_kpRule[3];
	
	/*********加权平均法解模糊*********/
    *ki = kiFuzzy[0] * fuzzy_kiRule[0] + kiFuzzy[1] * fuzzy_kiRule[1]\
			   + kiFuzzy[2] * fuzzy_kiRule[2] + kiFuzzy[3] * fuzzy_kiRule[3];
}

#if 0
float FuzzyPID_KpCalc(float f_e, float f_ec)
{
	int8_t num=0;
	float Kp_calcu=0.0f;
	float kpFuzzy[4] = {0.0f,0.0f,0.0f,0.0f}; //隶属于Kp的隶属程度
	
	FuzzySubjection(f_e, f_ec);
	
	/*********查询模糊规则表*********/
    num = kpRules[pe][pec];
    kpFuzzy[num] += eFuzzy[0] * ecFuzzy[0];
    num = kpRules[pe][pec + 1];
    kpFuzzy[num] += eFuzzy[0] * ecFuzzy[1];
    num = kpRules[pe + 1][pec];
    kpFuzzy[num] += eFuzzy[1] * ecFuzzy[0];
    num = kpRules[pe + 1][pec + 1];
    kpFuzzy[num] += eFuzzy[1] * ecFuzzy[1];
    /*********加权平均法解模糊*********/
    Kp_calcu = kpFuzzy[0] * fuzzy_kpRule[0] + kpFuzzy[1] * fuzzy_kpRule[1]\
			   + kpFuzzy[2] * fuzzy_kpRule[2] + kpFuzzy[3] * fuzzy_kpRule[3];
	
	return Kp_calcu;
}

float FuzzyPID_KiCalc(float f_e, float f_ec)
{
	int8_t num=0;
	float Ki_calcu=0.0f;
	float kiFuzzy[4] = {0.0f,0.0f,0.0f,0.0f}; //隶属于Kp的隶属程度
	
	FuzzySubjection(f_e, f_ec);
	
	/*********查询模糊规则表*********/
    num = kiRules[pe][pec];
    kiFuzzy[num] += eFuzzy[0] * ecFuzzy[0];
    num = kiRules[pe][pec + 1];
    kiFuzzy[num] += eFuzzy[0] * ecFuzzy[1];
    num = kiRules[pe + 1][pec];
    kiFuzzy[num] += eFuzzy[1] * ecFuzzy[0];
    num = kiRules[pe + 1][pec + 1];
    kiFuzzy[num] += eFuzzy[1] * ecFuzzy[1];
    /*********加权平均法解模糊*********/
    Ki_calcu = kiFuzzy[0] * fuzzy_kiRule[0] + kiFuzzy[1] * fuzzy_kiRule[1]\
			   + kiFuzzy[2] * fuzzy_kiRule[2] + kiFuzzy[3] * fuzzy_kiRule[3];
	
	return Ki_calcu;
}
#endif

void FuzzySubjection(float f_e, float f_ec)
{
	/*****误差E隶属函数描述*****/
	if(f_e<eRules[0])
	{
		eFuzzy[0] = 1.0f;
		pe = 0;
	}
	else if(eRules[0] <= f_e && f_e < eRules[1])
	{
		eFuzzy[0] = (eRules[1] - f_e) / (eRules[1] - eRules[0]);
		pe = 0;
	}
	else if(eRules[1] <= f_e && f_e < eRules[2])
	{
		eFuzzy[0] = (eRules[2] - f_e) / (eRules[2] - eRules[1]);
		pe = 1;
    }
    else if(eRules[2] <= f_e && f_e < eRules[3])
	{
		eFuzzy[0] = (eRules[3] - f_e) / (eRules[3] - eRules[2]);
		pe = 2;
    }
    else if(eRules[3] <= f_e && f_e < eRules[4])
	{
		eFuzzy[0] = (eRules[4] - f_e) / (eRules[4] - eRules[3]);
		pe = 3;
    }
    else if(eRules[4] <= f_e && f_e < eRules[5])
	{
		eFuzzy[0] = (eRules[5] - f_e) / (eRules[5] - eRules[4]);
		pe = 4;
    }
    else if(eRules[5] <= f_e && f_e < eRules[6])
	{
		eFuzzy[0] = (eRules[6] - f_e) / (eRules[6] - eRules[5]);
		pe = 5;
    }
    else
    {
        eFuzzy[0] = 0.0f;
		pe = 5;
    }
	eFuzzy[1] = 1.0f - eFuzzy[0];
	
	/*****误差变化率EC隶属函数描述*****/
    if(f_ec < ecRules[0])
	{
		ecFuzzy[0] = 1.0f;
		pec = 0;
    }
    else if(ecRules[0] <= f_ec && f_ec < ecRules[1])
	{
		ecFuzzy[0] = (ecRules[1] - f_ec) / (ecRules[1] - ecRules[0]);
		pec = 0;
    }

    else if(ecRules[1] <= f_ec && f_ec < ecRules[2])
	{
		ecFuzzy[0] = (ecRules[2] - f_ec) / (ecRules[2] - ecRules[1]);
		pec = 1;
    }
    else if(ecRules[2] <= f_ec && f_ec < ecRules[3])
	{
		ecFuzzy[0] = (ecRules[3] - f_ec) / (ecRules[3] - ecRules[2]);
		pec = 2;
    }
    else if(ecRules[3] <= f_ec && f_ec < ecRules[4])
	{
		ecFuzzy[0] = (ecRules[4] - f_ec) / (ecRules[4] - ecRules[3]);
		pec = 3;
    }
    else if(ecRules[4] <= f_ec && f_ec < ecRules[5])
	{
		ecFuzzy[0] = (ecRules[5] - f_ec) / (ecRules[5] - ecRules[4]);
		pec = 4;
    }

    else if(ecRules[5] <= f_ec && f_ec < ecRules[6])
	{
		ecFuzzy[0] = (ecRules[6] - f_ec) / (ecRules[6] - ecRules[5]);
		pec = 5;
    }
    else
    {
        ecFuzzy[0] = 0.0f;
		pec = 5;
    }
    ecFuzzy[1] = 1.0f - ecFuzzy[0];
}














