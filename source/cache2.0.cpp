#include<iostream>
#include<cstdlib>
#include<vector>
#include<string>
#include<cmath>
#include<fstream>
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

enum port_type{null_PORT,RO_PORT,WO_PORT,RW_PORT};	//�˿����� 

struct cache_t	//һ��cache�� 
{
  	bool valid;
  	long long tag;
  	long long time;
}RTcache[group_num][group_size];	

struct Set		 
{
  	int group_id[assoc];	//�����assoc�����λ�� 
  	int domain_id[assoc];
}set[sets];		//��һ�������¼ÿһ��set��ÿһ��assoc��Ӧ������cache�е�λ�� 


struct GroupPort		//��¼ÿ�������ϵĶ˿ڷֲ�
{
  	int port[64];			//��ʼ�ڷ�λ�� 
  	int shift_num;		//����λ�� 
}port_link[group_num];
 
int instr_num,wd_num,rd_num,wd_hit,rd_hit;  
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
void RT_shift(int group_id, int domain_id, bool is_write)
{
	int ans = 128, new_shift, dis;	
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
			} 
		}
	}

	this_one.shift_num += new_shift;		//�õ���λ�� 
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
	  		RT_shift(myGroup,myDomain,is_write);
      		return true;
    	}
  	}
  //cout<<"cache miss!\n";
  	int insertIndex=-1;
  	long long minTime = tick_time+1;
  	for(int i=0;i<assoc;i++)
  	{
	 	myGroup=set[myIndex].group_id[i];
	  	myDomain=set[myIndex].domain_id[i];
      	if(minTime>RTcache[myGroup][myDomain].time)	//�����validλ�ˣ���Ϊ��valid��timeһ����0 
      	{
			minTime = RTcache[myGroup][myDomain].time;
			insertIndex = i;							//�õ�Ҫ����ĵط����õ���LRU�滻���� 
      	}
  	}
  	int targetGroup=set[myIndex].group_id[insertIndex];	
  	int targetDomain=set[myIndex].domain_id[insertIndex];
  
  	RT_shift(targetGroup,targetDomain,is_write);
  
  	RTcache[targetGroup][targetDomain].valid = true;		//������Чλ 
  	RTcache[targetGroup][targetDomain].tag = myTag;		//��miss������д��cache
  	RTcache[targetGroup][targetDomain].time = tick_time;	//����ʱ�� 	
  
  	return false;
}

void readFile(string fileName)	//����trace 
{
  	ifstream fin(fileName.c_str());
  	char type;
  	long long addr, instr_time;
  	string a1, a2;
  	while(fin >> type >> addr >> a1 >> a2 >> instr_time)	//��������,����trace����ĵ�ַ��ʮ���Ƶģ� 
  	{
		//�����ж�ʱ���ֵ�ǰʱ���Ѿ�������һ��ָ�������ӳ������ˣ�����ֱ��ִ�ж����ȴ� 
		if(instr_time + instr_delay * CycleTicks > tick_time)
			tick_time = instr_time + instr_delay *CycleTicks;	
		instr_num++;
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
	
	/*�Ż�2���������Ķ��˿ڣ������ڶ���д��Ļ����ϣ�*/	
	/*
	for(int i=0;i<group_num;i++)
	{
		port_link[i].shift_num = 0;
		memset(port_link[i].port,0,sizeof(port_link[i].port));
		port_link[i].port[0] = RO_PORT;
		port_link[i].port[7] = RW_PORT;
		port_link[i].port[15] = RO_PORT;
		port_link[i].port[23] = RW_PORT;
		port_link[i].port[31] = RO_PORT;
		port_link[i].port[40] = RW_PORT;
		port_link[i].port[48] = RO_PORT;
		port_link[i].port[56] = RW_PORT;
		port_link[i].port[61] = RO_PORT;
	}
	*/
	
	/*�Ż�3����������д�˿ڣ�������д�ȶ���Ļ����ϣ�*/
	/*
	for(int i=0;i<group_num;i++)
	{
		port_link[i].shift_num = 0;
		memset(port_link[i].port,0,sizeof(port_link[i].port));
		port_link[i].port[5] = WO_PORT;
		port_link[i].port[11] = RW_PORT;
		port_link[i].port[25] = RW_PORT;
		port_link[i].port[31] = WO_PORT;
		port_link[i].port[38] = RW_PORT;
		port_link[i].port[50] = RW_PORT;
		port_link[i].port[62] = WO_PORT;
	}
	*/	
} 
void init_setmap()		//����set�Ļ��ֺͱ�ַ 
{
  /*baseline��set�������У���һ��group��8��������set*/
  	for(int i=0;i<sets;i++)
  	{
		for(int j=0;j<assoc;j++)
		{
			set[i].group_id[j]=i/assoc;
			set[i].domain_id[j]=(i%assoc)*assoc+j;
		}
  	}
}
void init_param()	//��ʼ������ȫ�ֱ��� 
{
	instr_num=wd_num=rd_num=wd_hit=rd_hit=0;  
	tick_time=0;	
	latency=0;
	latency_read=0;
	latency_write=0;	
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
