#include<iostream>
#include<cstdlib>
#include<vector>
#include<string>
#include<cmath>
#include<fstream>
#include<map>
using namespace std;

#define CycleTicks 1000
//cache���� 
#define sets 8192	//���� 
#define assoc 8		//������ 
#define blockSize 64	//cache���С 
#define index_bias 6
#define tag_bias index_bias+13

//group���� 
#define group_size 64
#define group_num 1024
 
//����ʱ���ӳ� 
#define rw_time 1		
#define move_time 1
#define instr_delay 6

//�����������Ĳ��� 
#define MT0 1
#define MT1 3
#define ST1 10
#define ST2 20

#define MIN(x,y)    ((x)<(y)?(x):(y))
 
map<pair<long long,char> , pair<long long, char> >nextinstr;
 
enum port_type{null_PORT,RO_PORT,WO_PORT,RW_PORT};	//�˿����� 
 
struct cache_t	//һ��cache�� 
{
  	bool valid;
  	long long tag;
  	long long time;
  	bool available;	//�Ƿ񿪷� 
}RTcache[group_num][group_size];	

struct Set				//һ��Set 
{
  	int group_id[assoc];	//�����assoc�����λ�� 
  	int domain_id[assoc];
  	int temp_miss;		//һ��ָ������Χ�ڵ�miss�� 
  	int temp_shift;		//һ��ָ������Χ�ڵ�shift�� 
  	int assoc_state;		//��ǰset��������״̬ 
}set[sets];		


struct GroupPort		//��¼ÿ�������ϵĶ˿ڷֲ�
{
  	int port[64];			//��ʼ�ڷ�λ�� 
  	int shift_num;		//����λ�� 
}port_link[group_num];
 
int instr_num,wd_num,rd_num,wd_hit,rd_hit;  

int last_addr=-1;	//ʵ�ʵ�ָ���ַ
char last_type;		//ʵ��ָ������ 
int last_shift=0,last_group=0;	//ʵ���ƶ��������ź��ƶ���λ�� 

int pred_addr=-1;	//Ԥ���ָ���ַ 
char pred_type;		//Ԥ���ָ������ 
int last_pre_shift=0, last_pre_group=0;	//Ԥ���ƶ��������ź��ƶ�λ�� 

int pred_num=0,pred_right_num=0;

long long tick_time=0;	//ʱ��tick
long long latency=0;	//�ܵ�ʱ���ӳ٣���λ��cycle 
long long latency_read=0, latency_write=0;

long long getIndex(long long addr)	//�õ�index 
{
  	long long res = addr>>index_bias;
  	res %= sets;
  	return res;
}

long long getTag(long long addr)	//�õ�tag 
{
  	long long res = addr>>tag_bias;
  	return res;
}
void change_assoc(int setIndex,int dst)		//Ŀǰֻ����ض���set�Ų���port�Ų�������� 
{
	int temp_group,temp_domain;
	int dir=(setIndex/1024)%2;		//���������� 

	if(dst==1)		//4->2  
	{
		if(dir==0)		//���ŵ�6��7���ۣ��ر������� 
		{
			for(int i=0;i<assoc-2;i++)
			{
				temp_group=set[setIndex].group_id[i];
				temp_domain=set[setIndex].domain_id[i];
				RTcache[temp_group][temp_domain].available=false;
				RTcache[temp_group][temp_domain].valid=false;
			}
			RTcache[set[setIndex].group_id[assoc-2]][set[setIndex].domain_id[assoc-2]].available=true;
			RTcache[set[setIndex].group_id[assoc-1]][set[setIndex].domain_id[assoc-1]].available=true;
		} 
		else			//���ŵ�0��1���ۣ��ر������� 
		{
			for(int i=2;i<assoc;i++)
			{
				temp_group=set[setIndex].group_id[i];
				temp_domain=set[setIndex].domain_id[i];
				RTcache[temp_group][temp_domain].available=false;
				RTcache[temp_group][temp_domain].valid=false;
			}
			RTcache[set[setIndex].group_id[0]][set[setIndex].domain_id[0]].available=true;
			RTcache[set[setIndex].group_id[1]][set[setIndex].domain_id[1]].available=true;
		}
	}
	else if(dst==2)		//8->4 || 2->4 
	{
		if(dir==0)		//���ŵ�4��5��6��7���ۣ��ر������� 
		{
			for(int i=0;i<assoc-4;i++)
			{
				temp_group=set[setIndex].group_id[i];
				temp_domain=set[setIndex].domain_id[i];
				RTcache[temp_group][temp_domain].available=false;
				RTcache[temp_group][temp_domain].valid=false;
			}
			for(int i=assoc-4;i<assoc;i++)
			{
				temp_group=set[setIndex].group_id[i];
				temp_domain=set[setIndex].domain_id[i];
				RTcache[temp_group][temp_domain].available=true;
			}
		} 
		else			//���ŵ�0��1��2��3���ۣ��ر������� 
		{
			for(int i=4;i<assoc;i++)
			{
				temp_group=set[setIndex].group_id[i];
				temp_domain=set[setIndex].domain_id[i];
				RTcache[temp_group][temp_domain].available=false;
				RTcache[temp_group][temp_domain].valid=false;
			}
			for(int i=0;i<4;i++)
			{
				temp_group=set[setIndex].group_id[i];
				temp_domain=set[setIndex].domain_id[i];
				RTcache[temp_group][temp_domain].available=true;
			}
		}
	}
	else		//4->8
	{
		//�������в� 
		for(int i=0;i<assoc;i++)
		{
			temp_group=set[setIndex].group_id[i];
			temp_domain=set[setIndex].domain_id[i];
			RTcache[temp_group][temp_domain].available=true;
		} 
	} 
}
void check_assoc()
{	
	for(int i=0;i<sets;++i)		//��������set��������������
	{
		switch(set[i].assoc_state)
		{
			/*
			case 0:
				if(set[i].temp_miss>MT1)
				{
					cout<<"Set "<<i<<": Miss reaching "<<MT1<<", changing from state 0 to 1 "<<endl;
					set[i].assoc_state=1;
					change_assoc(i,1);
				}
				break;
			*/
			case 1:
				if(set[i].temp_miss>MT1)
				{
					//cout<<"Set "<<i<<": Miss reaching "<<MT1<<", changing from state 1 to 2 "<<endl;
					set[i].assoc_state=2;
					change_assoc(i,2);
				}
				/*
				else if(set[i].temp_miss<MT0 && set[i].temp_shift>ST0)
				{
					cout<<"Set "<<i<<": Miss less than "<<MT0<<", shift reaching "<<ST0<<", changing from state 1 to 0 "<<endl;
					set[i].assoc_state=0;
					change_assoc(i,0);
				}*/
				break;
			case 2:
				if(set[i].temp_miss>MT1)
				{
					//cout<<"Set "<<i<<": Miss reaching "<<MT1<<", changing from state 2 to 3 "<<endl;
					set[i].assoc_state=3;
					change_assoc(i,3);
				}
				else if(set[i].temp_miss<MT0 && set[i].temp_shift>ST1)
				{
					//cout<<"Set "<<i<<": Miss less than "<<MT0<<", shift reaching "<<ST1<<", changing from state 2 to 1 "<<endl;
					set[i].assoc_state=1;
					change_assoc(i,1);
				}
				break;
			case 3:
				if(set[i].temp_miss<MT0 && set[i].temp_shift>ST2)
				{
					//cout<<"Set "<<i<<": Miss less than "<<MT0<<", shift reaching "<<ST2<<", changing from state 3 to 2 "<<endl;
					set[i].assoc_state=2;
					change_assoc(i,2);
				}
				break;
			default:
				cout<<" error "<<endl;	
		}
		set[i].temp_miss=set[i].temp_shift=0;		//��ԭ�ֲ�ͳ������ 
	}	
}
void RT_shift(int group_id, int domain_id, bool is_write,int setIndex,bool is_predict)
{
	int ans = 128, fine_ans=128, new_shift, dis,choice;	
	GroupPort &this_one = port_link[group_id];
	for(int i = 0;i < 64;i++)
	{	
		if((is_write && this_one.port[i]/2) || (!is_write && this_one.port[i]%2))//�ҵ�����ƥ��Ķ˿� 
		{
			dis = domain_id - (i + this_one.shift_num);
			if(abs(dis) < ans)	//ѡ����Ŀ������� 
			{
				ans = abs(dis);
				new_shift = dis;
				choice=i;
			} 
			else if(abs(dis) == ans && abs(this_one.shift_num+dis) < fine_ans)	//�Ż�������ʹ�����ع�ԭ��λ�� 
			{
				fine_ans = abs(this_one.shift_num+dis);
				new_shift = dis;
			}
		}
	}

	if(is_predict)		//��pre_shift�����ƣ�1��Ԥ���ƶ��ľ���ҪС��ͬʱ��ָ����ƶ�����
											//2�����߲���ͬһ������ 
	{					//�����last_group��last_shiftǡ�ɾ��ǲ���ִ�е�ָ����������ƶ�λ�� 
		if(abs(new_shift)>abs(last_shift) || group_id==last_group)
		{
			pred_addr=-1; 
			return ;
		}
		last_pre_shift = new_shift;	//��¼�˴�λ�� 
		last_pre_group = group_id;	//��¼�˴������� 
	}
	else
	{
		//cout<<"shift: "<<new_shift<<endl;
		last_shift = new_shift;	//��¼�˴�λ�� 
		last_group = group_id;	//��¼�˴������� 
	}
		
	this_one.shift_num += new_shift;		//�õ���λ�� 
	
	if(!is_predict)
	{
		tick_time += abs(new_shift) * move_time *CycleTicks;	//�����ƶ�ʱ�� 
		latency += abs(new_shift) * move_time;
	
		tick_time += rw_time *CycleTicks;		//���϶�дʱ�� 
	
		if(is_write)
		{
			latency_write += abs(new_shift) * move_time;
		}
		else
		{
			latency_read += abs(new_shift) * move_time;
			//latency_read += rw_time;
		}
		set[setIndex].temp_shift+=abs(new_shift);		//�������set��Ӧ��shift�� 
	}
	return;
}
bool visitCache(long long addr, bool is_write)
{
  	long long myIndex = getIndex(addr);
  	long long myTag = getTag(addr); 
  	//cout<<"Index & Tag: "<<myIndex<<" "<<myTag<<endl;
  	int myGroup, myDomain; 
  	for(int i=0;i<assoc;i++)
  	{
		myGroup=set[myIndex].group_id[i];
		myDomain=set[myIndex].domain_id[i];
    	if(RTcache[myGroup][myDomain].valid && RTcache[myGroup][myDomain].tag == myTag)	//hit
    	{ 
		  	//cout<<"cache hit!\n";
		  	//cout<<"Group & Domain: "<<myGroup<<" "<<myDomain<<endl;
		  	RTcache[myGroup][myDomain].time=tick_time;		//����ʱ�� 
	  		RT_shift(myGroup,myDomain,is_write,myIndex,0);
      		return true;
    	}
  	}
  	//cout<<"cache miss!\n";
  	set[myIndex].temp_miss++;	//��¼���set��miss���� 
  
  	int insertIndex=-1;
  	long long minTime = tick_time+1;
  	if((myIndex/1024)%2!=0)
  	{
  		for(int i=0;i<assoc;i++)
  		{
	  		myGroup=set[myIndex].group_id[i];
	  		myDomain=set[myIndex].domain_id[i];
      		if(RTcache[myGroup][myDomain].available && minTime>RTcache[myGroup][myDomain].time)	//�ҿ��õġ�����ʱ�������� 
      		{
				minTime = RTcache[myGroup][myDomain].time;
				insertIndex = i;							//�õ�Ҫ����ĵط����õ���LRU�滻���� 
      		}
		}
  	}
  	else
  	{
		for(int i=assoc-1;i>=0;i--)
  		{
	  		myGroup=set[myIndex].group_id[i];
	  		myDomain=set[myIndex].domain_id[i];
      		if(RTcache[myGroup][myDomain].available && minTime>RTcache[myGroup][myDomain].time)	//�ҿ��õġ�����ʱ�������� 
      		{
				minTime = RTcache[myGroup][myDomain].time;
				insertIndex = i;							//�õ�Ҫ����ĵط����õ���LRU�滻���� 
      		}
		}
  	}
  	int targetGroup=set[myIndex].group_id[insertIndex];	
  	int targetDomain=set[myIndex].domain_id[insertIndex];
  	//cout<<"Group & Domain: "<<targetGroup<<" "<<targetDomain<<endl;
  
  	RT_shift(targetGroup,targetDomain,is_write,myIndex,0);
  
  	RTcache[targetGroup][targetDomain].valid = true;		//������Чλ 
  	RTcache[targetGroup][targetDomain].tag = myTag;		//��miss������д��cache
  
  	RTcache[targetGroup][targetDomain].time = tick_time;	//����ʱ�� 	
  
  	return false;
}
void pre_visitCache(long long addr, bool is_write)		//ֻ��Ԥ�⣬������������cache 
{
  	long long myIndex = getIndex(addr);
  	long long myTag = getTag(addr); 
  	//cout<<"Index & Tag: "<<myIndex<<" "<<myTag<<endl;
  	int myGroup, myDomain; 
  	for(int i=0;i<assoc;i++)
  	{
		myGroup=set[myIndex].group_id[i];
		myDomain=set[myIndex].domain_id[i];
    	if(RTcache[myGroup][myDomain].valid && RTcache[myGroup][myDomain].tag == myTag)	//hit
    	{ 
	  		//cout<<"cache hit!\n";
	  		//cout<<"Group & Domain: "<<myGroup<<" "<<myDomain<<endl;
	  		RT_shift(myGroup,myDomain,is_write,myIndex,1);
      		return ; 
    	}
  	}
  	//cout<<"cache miss!\n";
  	int insertIndex=-1;
  	long long minTime = tick_time+1;
  	if((myIndex/1024)%2!=0)
  	{
  		for(int i=0;i<assoc;i++)
  		{
	  		myGroup=set[myIndex].group_id[i];
	  		myDomain=set[myIndex].domain_id[i];
      		if(RTcache[myGroup][myDomain].available && minTime>RTcache[myGroup][myDomain].time)	//�ҿ��õġ�����ʱ�������� 
      		{
				minTime = RTcache[myGroup][myDomain].time;
				insertIndex = i;							//�õ�Ҫ����ĵط����õ���LRU�滻���� 
      		}
		}
  	}
  	else
  	{
		for(int i=assoc-1;i>=0;i--)
  		{
	  		myGroup=set[myIndex].group_id[i];
	  		myDomain=set[myIndex].domain_id[i];
      		if(RTcache[myGroup][myDomain].available && minTime>RTcache[myGroup][myDomain].time)	//�ҿ��õġ�����ʱ�������� 
      		{
				minTime = RTcache[myGroup][myDomain].time;
				insertIndex = i;							//�õ�Ҫ����ĵط����õ���LRU�滻���� 
      		}
		}
  	}
  
  	int targetGroup=set[myIndex].group_id[insertIndex];	
  	int targetDomain=set[myIndex].domain_id[insertIndex];
  	//cout<<"Group & Domain: "<<targetGroup<<" "<<targetDomain<<endl;
  
  	RT_shift(targetGroup,targetDomain,is_write,myIndex,1);  
  	return ;		
} 
void resetTwoGroup(int IdleCycle)	
{
	if(IdleCycle == 0) 
		return;
	int firstIndex = 0,secondIndex = 0;
	for(int i = 0;i< group_num;i++)		//ѡȡ��ǰƫ����������group��ԭ 
	{
		if(abs(port_link[i].shift_num) >= abs(port_link[firstIndex].shift_num))
		{
			secondIndex = firstIndex;	
			firstIndex = i;
		} else if(abs(port_link[i].shift_num) >= abs(port_link[secondIndex].shift_num)
				|| firstIndex == secondIndex)
		{
			secondIndex = i;
		} 
	}
	int t1 = port_link[firstIndex].shift_num,t2 = port_link[secondIndex].shift_num;
	port_link[firstIndex].shift_num = t1 <= 0 ? t1 + MIN(-t1,IdleCycle) : t1 - MIN(t1,IdleCycle);
	port_link[secondIndex].shift_num = t2 <= 0 ? t2 + MIN(-t2,IdleCycle) : t2 - MIN(t2,IdleCycle);
	return;
}

void resetSingleGroup(int groupException,int IdleCycle)
{
	if(IdleCycle == 0) 
		return;
	int firstIndex = 0;
	for(int i = 0;i<group_num;i++)	//�ƶ����˵�ǰ������������� 
	{
		if(i != groupException && abs(port_link[i].shift_num) >= abs(port_link[firstIndex].shift_num))
			firstIndex = i;
	}
	int t1 = port_link[firstIndex].shift_num;
	port_link[firstIndex].shift_num = t1 <= 0 ? t1 + MIN(-t1,IdleCycle) : t1 - MIN(t1,IdleCycle);
	return;
}
void readFile(string fileName)	//����trace 
{
  	ifstream fin(fileName.c_str());
  	char type;
  	long long addr, instr_time, timeRecord=0;
  	string a1, a2;
  	int extra_shift=0,extra_latency=0,temp_latency=0;
  	while(fin >> type >> addr >> a1 >> a2 >> instr_time)	//��������,����trace����ĵ�ַ��ʮ���Ƶģ� 
  	{
		instr_num++; 
		if(pred_addr!=-1)
		{
			pred_num++;
			if(pred_addr==addr && pred_type==type)	//Ԥ����ȷ 
			{
				//cout<<instr_num<<" "<<addr<<endl; 
				pred_right_num++;
			}
		} 
		
		//����ָ��֮��Ŀ���ʱ�䲢�л�ԭ����������������ʱ���ӳ� 
    	if(instr_time + instr_delay * CycleTicks > tick_time)
    	{
			resetTwoGroup((instr_time + instr_delay * CycleTicks - tick_time)/CycleTicks);
    		tick_time = instr_time + instr_delay *CycleTicks;
		}
	
		//���뵱ǰָ��ִ�� 
		timeRecord = tick_time;
	
    	switch(type)
    	{
      		case 'r':
				rd_num++;
				if(visitCache(addr,0))
	  				rd_hit++;
				break;
      		case 'w':
	  		case 'u':
				wd_num++;
				if(visitCache(addr,1))
	  				wd_hit++;
      			break;
      		default:
				cout << "error"<< endl;
   		}
    	if(instr_num%500==0)		//ÿ��500��ָ�����Ƿ�Ҫ���������� 
			check_assoc();	
		
		//Ԥ�⵱ǰָ�����һ����Ԥ�ƶ�������ǿ���趨�˴��뱾��ָ�����ɣ����򲻽���Ԥ�ƶ�
		if(last_addr!=-1)	
			nextinstr[pair<long long,char>(last_addr,last_type)]=pair<long long,char>(addr,type);	//��¼����ָ��� 
		last_addr=addr;
		last_type=type;
	
		if(nextinstr.count(pair<long long,char>(addr,type))!=0)	//����ӦԤ�� 
		{
			pred_addr=nextinstr[pair<long long,char>(addr,type)].first;
			pred_type=nextinstr[pair<long long,char>(addr,type)].second;
			//cout<<addr<<" "<<type<<" "<<pred_addr<<" "<<pred_type<<endl;
			if(pred_type=='r')
				pre_visitCache(pred_addr,0);
			else
				pre_visitCache(pred_addr,1);
		}	 
		else
		{
			pred_addr=-1;
			//û�н���ָ��Ԥ�⣬������ָ��ִ��ʱ�䲢�л�ԭһ��������������ʱ���ӳ� 
  			resetSingleGroup(last_group,(tick_time - timeRecord) / CycleTicks);
		}
  	}
}
void init_RTcache()		//��ʼ��cache 
{
	for(int i=0;i<group_num;i++)
		for(int j=0;j<group_size;j++)
		{
			RTcache[i][j].tag=0;
			RTcache[i][j].valid=false;
			RTcache[i][j].time=0;
			RTcache[i][j].available=false;
		}
	for(int i=0;i<sets;i++)		//��ʼ���������� 
	{
		int dir=(i/1024)%2;
		if(dir==0)				//ֻ���ŵ�6��7���� 
		{
			RTcache[set[i].group_id[assoc-2]][set[i].domain_id[assoc-2]].available=true;
			RTcache[set[i].group_id[assoc-1]][set[i].domain_id[assoc-1]].available=true;
		}
		else					//ֻ���ŵ�0��1���� 
		{
			RTcache[set[i].group_id[0]][set[i].domain_id[0]].available=true;
			RTcache[set[i].group_id[1]][set[i].domain_id[1]].available=true;
		}
	}
}
void init_portmap()		//����ÿ��group�˿ڸ������ڷ����
{
	/*�Ż�1���ö˿ڽ�64��domain���ƽ���ָ*/ 
	
	for(int i=0;i<group_num;i++)
	{
		port_link[i].shift_num = 0;
		memset(port_link[i].port,0,sizeof(port_link[i].port));
		port_link[i].port[7] = RW_PORT;
		port_link[i].port[23] = RW_PORT;
		port_link[i].port[40] = RW_PORT;
		port_link[i].port[56] = RW_PORT;
	}
} 
void init_setmap()		//����set�Ļ��ֺͱ�ַ 
{
  	/*�Ż���set�������У�ÿ��set���1��group*/
  
  	int span=1;
  	int set_start, domain_start;
  	for(int i=0;i<sets;i++)
  	{
		set_start=(i*span)%group_num;
		domain_start=(i/(group_num/span))*(assoc/span);
		for(int j=0;j<assoc;j++)
		{
			set[i].group_id[j]=set_start+j%span;
			set[i].domain_id[j]=domain_start+j/span;
		}
  	}
}
void init_param()
{
	instr_num=wd_num=rd_num=wd_hit=rd_hit=0;  
	tick_time=0;	
	latency=0;
	latency_read=0;
	latency_write=0;	
	
	nextinstr.clear();
	last_addr=-1;	
	last_shift=0;
	last_group=0;	 

	pred_addr=-1;	
	last_pre_shift=0;
	last_pre_group=0;
	
	pred_num=0;
	pred_right_num=0;	
}
int main()
{
  
  	string trace[]={"401.bzip2","403.gcc","410.bwaves","435.gromacs","450.soplex",
  	"453.povray","456.hmmer","471.omnetpp"};
  	char output[100];
  	memset(output,'\0',sizeof(output));
  	for(int i=0;i<8;i++)
  	{
		cout<<i<<endl;
		init_param();
  		init_RTcache();
  		init_portmap();
  		init_setmap();
  		readFile("D:\\������\\�������֯����ϵ�ṹ\\����ҵ\\trace\\Result\\"+trace[i]+".trace");	
  		strcpy(output,"D:\\������\\�������֯����ϵ�ṹ\\����ҵ\\Red & Yuan\\Result2.0\\");
		strcat(output,trace[i].c_str());
  		strcat(output,".result.txt");
		ofstream fout(output);
  		fout<<"miss rate:"<<1-(double)(rd_hit+wd_hit)/(rd_num+wd_num)<<endl;
  		fout<<"on read: "<<latency_read<<" ";
  		fout<<"on write: "<<latency_write<<" ";
  		fout<<"total latency: "<<latency<<endl;  	
  	}
  	//system("pause");
  	return 0;
}

