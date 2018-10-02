#include <stdio.h>
#include <vector>
#include <stdlib.h>
#include <fstream>
#include <iostream>
#include <time.h>
#include <string.h>

using namespace std;

#define MAXLOOP 20000
#define OUTPUTLOOP 500
struct Likelihood  //to save state and weight of likelihood method
{
	vector<bool> u;
	double w;
};

struct Gibbs_X //to save account and probability distribution of Gibbs method
{
	double var_T,var_F;
	int count_T,count_F;
};

class Inference
{
public:	
	Inference(char* filepath);	//constructor

private:
	int N,T;
	vector<bool> umbrella;		//save to input evidence variables
	vector<Likelihood> likelihood_S;
	vector<Gibbs_X> Gibbs_S;
	int readfile(char* filepath);		//read input file
	void print();
	void compute_LH_Distribution();		
	void solve();
	void likelihood_solver();
	void Gibbs_solver();
	bool compare_sample(vector<bool> u1,vector<bool> u2);
};

void Inference::Gibbs_solver()
{
	vector<bool> r_t;
	srand((unsigned)time(0));
	int p,n=0;
	bool var;
	Gibbs_X X_t;
	for(int i=0;i<T;i++)		//initialise the non-obvserved variables
	{
		
		p=rand()%2;	
		if(p==0)
		{
			var=0;
			X_t.var_F=0.8;
			X_t.var_T=0.2;
			X_t.count_T=0;
			X_t.count_F=1;
		}
		else
		{
			var=1;
			X_t.var_F=1;
			X_t.var_T=1;
			X_t.count_T=1;
			X_t.count_F=0;
		}
		Gibbs_S.push_back(X_t);
		r_t.push_back(var);
	}
	double p_T,p_F,temp_p,result_T;
	/*ofstream f;
	f.open("Gibbsresult.txt");*/
	while(n<MAXLOOP)		//for each interation
	{
		
		for(int i=0;i<T;i++)		//for each dimension,compute it's distribution
		{
			p_T=1;
			p_F=1;
			
			if(umbrella[i])
			{
				p_T*=0.9;
				p_F*=0.2;
			}
			else
			{
				p_T*=0.1;
				p_F*=0.8;
			}
			if(i==T-1)
			{					
				if(r_t[i-1])
				{
					p_T*=0.7;
					p_F*=0.3;
				}
				else
				{
					p_T*=0.3;
					p_F*=0.7;
				}
			}
			else if(i==0)
			{
				p_T*=0.2;
				p_F*=0.8;
				if(T>1)
				{
					if(r_t[i+1])
					{
						p_T*=0.7;
						p_F*=0.3;
					}
					else
					{
						p_T*=0.3;
						p_F*=0.7;
					}
				}
			}
			else 
			{
				if(r_t[i-1])
				{
					p_T*=0.7;
					p_F*=0.3;
				}
				else
				{
					p_T*=0.3;
					p_F*=0.7;
				}
				if(r_t[i+1])
				{
					p_T*=0.7;
					p_F*=0.3;
				}
				else
				{
					p_T*=0.3;
					p_F*=0.7;
				}
			}
			temp_p=p_T+p_F;
			p_T=p_T/temp_p;
			p_F=1-p_T;
			p=rand()%1000;			
			if(p<p_T*1000)		//generate new random variable 
			{
				r_t[i]=1;
				Gibbs_S[i].var_T=p_T;
				Gibbs_S[i].var_F=p_F;
				Gibbs_S[i].count_T++;			
			}
			else
			{
				r_t[i]=0;
				Gibbs_S[i].var_T=p_T;
				Gibbs_S[i].var_F=p_F;
				Gibbs_S[i].count_F++;	
			}

		}
		//for(int i=0;i<T;i++)cout<<Gibbs_S[i].var_T<<" "<<Gibbs_S[i].var_F<<" Gibbs "<<i<<endl;
		/*if(n%OUTPUTLOOP==0)
		{
			result_T=(double)Gibbs_S[T-1].count_T/(double)(Gibbs_S[T-1].count_T+Gibbs_S[T-1].count_F);
			f<<result_T<<endl;
		}*/

		n++;
	}
	
	result_T=(double)Gibbs_S[T-1].count_T/(double)(Gibbs_S[T-1].count_T+Gibbs_S[T-1].count_F);	//computer the posterior probability
	
	cout<<result_T<<" "<<1-result_T<<" Gibbs"<<endl;
	/*f.close();*/
}

void Inference::likelihood_solver()
{
	Likelihood l_t;
	srand((unsigned)time(0));
	int n=0,p;
	bool var;
	/*ofstream f;
	f.open("LHresult.txt");*/
	while(n<MAXLOOP)	//for each interation
	{
		l_t.u.clear();
		l_t.w=1;
		for(int i=0;i<T;i++)	//for each dimension, sampling randomly with it's distribution
		{
			p=rand()%10;
			if(i==0)
			{
				if(p<2)
				{
					var=1;
					if(umbrella[i])l_t.w*=0.9;
					else l_t.w*=0.1;
				}
				else
				{
					var=0;
					if(umbrella[i])l_t.w*=0.2;
					else l_t.w*=0.8;
				}
			}
			else
			{
				if(l_t.u[i-1])
				{
					if(p<7)
					{
						var=1;
						if(umbrella[i])l_t.w*=0.9;
						else l_t.w*=0.1;
					}
					else
					{
						var=0;
						if(umbrella[i])l_t.w*=0.2;
						else l_t.w*=0.8;
					}
				}
				else
				{
					if(p<3)
					{
						var=1;
						if(umbrella[i])l_t.w*=0.9;
						else l_t.w*=0.1;
					}
					else
					{
						var=0;
						if(umbrella[i])l_t.w*=0.2;
						else l_t.w*=0.8;
					}
				}

			}
			l_t.u.push_back(var);			
		}
		if(likelihood_S.size()==0)		//save the state to the list
		{
			likelihood_S.push_back(l_t);
		}
		else		
		{
			for(int i=0;i<likelihood_S.size();i++)		
			{
				if(compare_sample(likelihood_S[i].u,l_t.u))
				{
					likelihood_S[i].w+=l_t.w;
					break;
				}
				if(i==(likelihood_S.size()-1))
				{
					likelihood_S.push_back(l_t);
				}
			}		
		}
		/*if(n%OUTPUTLOOP==0)
		{
			double temp=0,total=0,result_T,result_F;
			for(int i=0;i<likelihood_S.size();i++)
			{
				if(likelihood_S[i].u[T-1])temp+=likelihood_S[i].w;
				total+=likelihood_S[i].w;
			}
			result_T=temp/total;
			result_F=1-result_T;
			f<<result_T<<endl; 
		}		*/
		n++;
	}
	/*f.close();*/
	compute_LH_Distribution();
}

void Inference::solve()
{
	likelihood_solver();
	Gibbs_solver();
}

void Inference::compute_LH_Distribution()
{
	double temp=0,total=0,result_T,result_F;		//compute the distribution of likelihood
	for(int i=0;i<likelihood_S.size();i++)
	{
		if(likelihood_S[i].u[T-1])temp+=likelihood_S[i].w;
		total+=likelihood_S[i].w;
	}
	result_T=temp/total;
	result_F=1-result_T;
	cout<<result_T<<" "<<result_F<<" Likelihood"<<endl; 
}

bool Inference::compare_sample(vector<bool> u1,vector<bool> u2)	//compare the two state , if they are exactly same ,return true
{
	if(u1.size()!=u2.size())return 0;
	for(int i=0;i<u1.size();i++)
	{
		if(u1[i]!=u2[i])return 0;
	}
	return 1;
}



Inference::Inference(char* filepath)
{
	if(int ret=readfile(filepath)<0)
	{
		cout<<"File "<<filepath<<" open failed!"<<endl;
		exit(1);
	}
	solve();
	//print();
}


int Inference::readfile(char* filepath)	//read the input file
{
	ifstream f;
	f.open(filepath);
	if(!f.is_open())return -1;
	char str[10];
	int count=0;
	bool temp;
	while(!f.eof())
	{
		f>>str;
		if(strcmp(str,"")==0)break;
		if(strcmp(str,"1")==0)temp=1;
		else temp=0;		
		umbrella.push_back(temp);
		count++;
	}
	T=count;
	f.close();
}



void Inference::print()
{
	for(int i=0;i<umbrella.size();i++)
	{
		cout<<umbrella[i]<<" ";
	}
	cout<<endl;
	system("pause");
}


int main(int argc,char** argv)
{
	if(argc!=2)
	{
		cout<<"Invalid command argument"<<endl;
		return -1;
	}
	Inference inference(argv[1]);		//generate new inference object
}
