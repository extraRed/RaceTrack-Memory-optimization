#include<iostream>
#include<cstdlib>
#include<vector>
#include<string>
#include<cmath>
#include<fstream>
using namespace std;

#define CycleTicks 1000
//cache参数 
#define sets 8192	//组数 
#define assoc 8		//相连度 
#define blockSize 64	//cache块大小 
#define index_bias 6
#define tag_bias index_bias+13

//group参数 
#define group_size 64
#define group_num 1024
 
//各种时间延迟 
#define rw_time 1		
#define move_time 1
#define instr_delay 6

enum port_type{null_PORT,RO_PORT,WO_PORT,RW_PORT};	//端口类型 

struct cache_t	//一个cache块 
{
  	bool valid;
  	long long tag;
  	long long time;
}RTcache[group_num][group_size];	

struct Set		 
{
  	int group_id[assoc];	//代表第assoc个块的位置 
  	int domain_id[assoc];
}set[sets];		//用一个数组记录每一个set的每一个assoc对应在新型cache中的位置 


struct GroupPort		//记录每个条带上的端口分布
{
  	int port[64];			//初始摆放位置 
  	int shift_num;		//整体位移 
}port_link[group_num];
 
int instr_num,wd_num,rd_num,wd_hit,rd_hit;  
long long tick_time=0;	//时钟tick
long long latency=0;	//总的时间延迟，单位是cycle 
long long latency_read=0, latency_write=0;

long long getIndex(long long addr)	//得到index 
{
  	long long res = addr>>index_bias;
  	res %= sets;
  	return res;
}

long long getTag(long long addr)	//得到tag 
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
		if((is_write && this_one.port[i]/2) || (!is_write && this_one.port[i]%2))//找到类型匹配的端口 
		{
			dis = domain_id - (i + this_one.shift_num);
			if(abs(dis) < ans)	//选择离目标最近的 
			{
				ans = abs(dis);
				new_shift = dis;
			} 
		}
	}

	this_one.shift_num += new_shift;		//得到新位移 
	tick_time += abs(new_shift) * move_time *CycleTicks;	//加上移动时间 
	latency += abs(new_shift) * move_time;
	
	tick_time += rw_time *CycleTicks;		//加上读写时间 
	
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
	  		RTcache[myGroup][myDomain].time=tick_time;		//更新时间 
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
      	if(minTime>RTcache[myGroup][myDomain].time)	//不检查valid位了，因为不valid的time一定是0 
      	{
			minTime = RTcache[myGroup][myDomain].time;
			insertIndex = i;							//得到要插入的地方，用的是LRU替换策略 
      	}
  	}
  	int targetGroup=set[myIndex].group_id[insertIndex];	
  	int targetDomain=set[myIndex].domain_id[insertIndex];
  
  	RT_shift(targetGroup,targetDomain,is_write);
  
  	RTcache[targetGroup][targetDomain].valid = true;		//设置有效位 
  	RTcache[targetGroup][targetDomain].tag = myTag;		//将miss的数据写入cache
  	RTcache[targetGroup][targetDomain].time = tick_time;	//更新时间 	
  
  	return false;
}

void readFile(string fileName)	//读入trace 
{
  	ifstream fin(fileName.c_str());
  	char type;
  	long long addr, instr_time;
  	string a1, a2;
  	while(fin >> type >> addr >> a1 >> a2 >> instr_time)	//读入请求,好像trace里面的地址是十进制的？ 
  	{
		//假设判断时发现当前时间已经超过下一条指令六个延迟周期了，可以直接执行而不等待 
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
void init_RTcache()		//初始化cache 
{
	for(int i=0;i<group_num;i++)
		for(int j=0;j<group_size;j++)
		{
			RTcache[i][j].tag=0;
			RTcache[i][j].valid=false;
			RTcache[i][j].time=0;
		}
}
void init_portmap()		//定义每个group端口个数及摆放情况
{
	/*优化1：让端口将64个domain相对平均分割开*/ 
	
	for(int i=0;i<group_num;i++)
	{
		port_link[i].shift_num = 0;
		memset(port_link[i].port,0,sizeof(port_link[i].port));
		port_link[i].port[7] = RW_PORT;
		port_link[i].port[23] = RW_PORT;
		port_link[i].port[40] = RW_PORT;
		port_link[i].port[56] = RW_PORT;
	}
	
	/*优化2：加入更多的读端口（建立在读比写多的基础上）*/	
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
	
	/*优化3：加入更多的写端口（建立在写比读多的基础上）*/
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
void init_setmap()		//定义set的划分和编址 
{
  /*baseline：set纵向排列，即一个group放8个连续的set*/
  	for(int i=0;i<sets;i++)
  	{
		for(int j=0;j<assoc;j++)
		{
			set[i].group_id[j]=i/assoc;
			set[i].domain_id[j]=(i%assoc)*assoc+j;
		}
  	}
}
void init_param()	//初始化各种全局变量 
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
  		readFile("D:\\大三上\\计算机组织与体系结构\\大作业\\trace\\Result\\"+trace[i]+".trace");	
  		strcpy(output,"D:\\大三上\\计算机组织与体系结构\\大作业\\Red & Yuan\\Result2.0\\");
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
