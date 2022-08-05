//#include <bits/stdc++.h>

#include <iostream>
#include <vector>
#include <string>
#include <numeric>
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <random>

using namespace std;

const int MOD = 998'244'353;
//const int MOD = 1000'000'007;

const int INF = (1<<30)-1;
const long long LINF = (1LL<<62)-1;

#define rep(i, n) for (int i = 0; i < (int)(n); i++)
#define repi(i, a, n) for (int i = a; i < (int)(n); i++)
#define repll(i, n) for (ll i = 0; i < (ll)(n); i++)
#define repill(i, a, n) for (ll i = a; i < (ll)(n); i++)
#define pb(a) push_back(a)
#define PS(a) cout<<(a)<<" ";
#define PL(a) cout<<(a)<<endl;
//#define ONLINE_JUDGE 1
#ifdef ONLINE_JUDGE
    #define ES(a) while(0){}
    #define EL(a) while(0){}
#else
    #define NAME(a) #a
    #define ES(a) cerr<<NAME(a)<<": "<<(a)<<" ";
    #define EL(a) cerr<<NAME(a)<<": "<<(a)<<endl;
#endif
#define END(a) {PL(a) return;}
#define RES(a) cerr<<"\r"<<NAME(a)<<": "<<(a)<<"   ";
#define fi first
#define se second
#define ALL(a)  (a).begin(),(a).end()
#define RALL(a)  (a).rbegin(),(a).rend()
#define SORT(a) sort(a.begin(), a.end());
#define REVERSE(a) reverse(a.begin(), a.end());
#define ERASE(a) a.erase(unique(a.begin(), a.end()), a.end());
template< typename T >ostream &operator<<(ostream &os, const vector< T > &v) {for(int i = 0; i < (int) v.size(); i++) {os << v[i] << (i + 1 != (int) v.size() ? " " : "");}return os;}
template< typename T >istream &operator>>(istream &is, vector< T > &v) {for(T &in : v) is >> in;return is;}
template <class T, class U> void chmin(T& t, const U& u) {if (t > u) t = u;}
template <class T, class U> void chmax(T& t, const U& u) {if (t < u) t = u;}

struct NormalDistRandom{
    const unsigned SEED;
    mt19937 engine;
    // default_random_engine engine;

    double mean = 0.;
    double stddev = 1.;
    normal_distribution<> dist;

    NormalDistRandom(double mean, double stddev):SEED(random_device()()), engine(SEED)
    , mean(mean), stddev(stddev), dist(normal_distribution<>(mean, stddev)) {}
    double operator()(){
        return dist(engine);
    }
    /*
    正規分布 : 平均，　標準偏差
    NormalDistRandom ndrand(0.0, 1.0);
    cout<< ndrand() <<endl;

    */
};

vector<string> readfile(string filename){ 
    vector<string> ret;
    ifstream ifs(filename); 
    string line; 
    while(getline(ifs,line)){ 
        ret.push_back(line); 
    } 
    return ret; 
}

vector<string> split(const string& s_in, string split=","){ 
    vector<string> ret; 
    string tmp; 
    for(int i=0;i<(int)s_in.size();i++){
        bool found = false; 
        for(int j=0;j<(int)split.size();j++){ 
            if(s_in[i]==split[j]){ 
                if(tmp!=""){ret.push_back(tmp);} 
                tmp = ""; 
                found = true; 
                break; 
            }
        } 
        if(!found) tmp+=s_in[i];
    } 
    if(tmp!="") ret.push_back(tmp); 
    return ret;
}

//vector<string> file = readfile("text.csv");
//string line = readline();
//vector<string> sp = split("a,b,av. d,tf", ",.");
//vector<long long> isp = int_split("95945435, 1000000007,   4566.6", ",");
//vector<double> dsp = double_split("999.4, 3 33,5, 111.3", ", ");
//for(auto s: dsp) cout<<s<<endl;

struct Vec{
    double x,y,z;
    int id;
    Vec(double x, double y, double z, int id, double scale = 1.0):x(x*scale), y(y*scale), z(z*scale), id(id){}

    double& get(int i){
        if(i==0) return x;
        if(i==1) return y;
        return z;
    }

    Vec operator+(const Vec& A){
        return Vec(x+A.x, y+A.y, z+A.z, -1);
    }

    Vec operator-(const Vec& A){
        return Vec(x-A.x, y-A.y, z-A.z, -1);
    }

    Vec operator*(int a){
        return Vec(x*a, y*a, z*a, -1);
    }

    Vec operator/(int a){
        if(a==0) return *(this);
        return Vec(x/a, y/a, z/a, -1);
    }

    bool operator<(const Vec& A){
        if(x!=A.x) return x<A.x;
        if(y!=A.y) return y<A.y;
        return z<A.z;
    }

    bool operator==(const Vec& A){
        if (abs(A.x - x) > 1e-6) return false;
        if (abs(A.y - y) > 1e-6) return false;
        if (abs(A.z - z) > 1e-6) return false;
        return true;
    }
};

double distance(const Vec& A, const Vec& B){
    double ret = sqrt((A.x-B.x)*(A.x-B.x)
                    + (A.y-B.y)*(A.y-B.y)
                    + (A.z-B.z)*(A.z-B.z));
    return ret;
}

struct finfo{
    int v[3];
    int id;
    finfo(int v0, int v1, int v2, int id):id(id){
        v[0]=v0; v[1]=v1; v[2]=v2;
        sort(&v[0], &v[2]);
    }
};




struct PCDMaker{
    //頂点座標
    vector<Vec> Vp;
    //面法線
    vector<Vec> Vn;
    //面情報
    vector<finfo> Fs;
    //計算結果出力用
    vector<Vec> resultVp;
    //採用される点
    vector<bool> ac;

    double resolution = 0.001;
    double mx =  1e10, my =  1e10, mz =  1e10;
    double Mx = -1e10, My = -1e10, Mz = -1e10;
    
    void input(string objname, double scale = 0.001){
        //scale=0.001 -> [mm]->[m]
        ///--------------------ファイル読み込み，抽出-----------------------///
        auto file = readfile(objname);

        Vp.reserve(file.size());
        Vn.reserve(file.size());
        Fs.reserve(file.size());

        rep(i, file.size()){
            auto line = split(file[i], " ");
            //頂点座標
            if(line[0]=="v"){
                Vp.push_back(Vec(stod(line[1]), stod(line[2]), stod(line[3]), Vp.size(), scale));
            }
            //面法線
            else if(line[0]=="vn"){
                Vn.push_back(Vec(stod(line[1]), stod(line[2]), stod(line[3]), Vn.size(), scale));
            }
            //面情報
            else if(line[0]=="f"){
                //f 1041//40 1044//40 1043//40
                Fs.push_back(finfo(stoi(line[1])-1, stoi(line[2])-1, stoi(line[3])-1, Fs.size()));
            }
        }
        ES(objname) cerr<<"  finished read"<<endl;
        EL(Vp.size())
        EL(Vn.size())
        EL(Fs.size())
        cerr<<endl;
    }

    void calc(double resolution_ = 0.001){
        resolution = resolution_;
        //resolution:mm



        ///--------------------面補填-----------------------///
        rep(i, Fs.size()){
            //頂点番号
            int* v = Fs[i].v;

            //内部埋め
            Vec vec1 = Vp[v[0]];
            Vec vec2 = Vp[v[0]];
            double dist1 = distance(Vp[v[0]], Vp[v[1]]);
            double dist2 = distance(Vp[v[0]], Vp[v[2]]);
            int num = max(dist1, dist2) /resolution;
            repi(j, 1, num+1){
                vec1 = Vp[v[0]] + (Vp[v[1]] - Vp[v[0]])*j/num;
                vec2 = Vp[v[0]] + (Vp[v[2]] - Vp[v[0]])*j/num;
                vec1.id = Vp.size();
                vec2.id = Vp.size()+1;
                Vp.push_back(vec1);
                Vp.push_back(vec2);

                Vec vec12 = vec1;
                double dist12 = distance(vec1, vec2);
                int num12 = dist12/resolution;
                repi(k, 1, num12+1){
                    vec12 = vec1 + (vec2-vec1) * k / num12;
                    vec12.id = Vp.size();
                    Vp.push_back(vec12);
                }
            }

        }
        cerr<<"internal compensate finished"<<endl;
        EL(Vp.size())
        cerr<<endl;
    }

    void calcMaxMin(){
        ///--------------------各軸方向最大最小-----------------------///
        mx =  1e10, my =  1e10, mz =  1e10;
        Mx = -1e10, My = -1e10, Mz = -1e10;
        rep(i,Vp.size()){
            chmin(mx, Vp[i].x);
            chmax(Mx, Vp[i].x);
            chmin(my, Vp[i].y);
            chmax(My, Vp[i].y);
            chmin(mz, Vp[i].z);
            chmax(Mz, Vp[i].z);
        }
        cerr<<"calc min and Max for each direction"<<endl;
        ES(mx) EL(Mx)
        ES(my) EL(My)
        ES(mz) EL(Mz)
        cerr<<endl;        
    }

    void checkID(){
        ///--------------------id重複欠損確認-----------------------///
        bool id_ok = true;
        vector<bool> exist(Vp.size());
        rep(i,Vp.size()){
            if(Vp[i].id<exist.size()){
                if(!exist[Vp[i].id]) exist[Vp[i].id] = true;
                else{
                    cerr<<Vp[i].id<<"  found several times"<<endl;
                    id_ok = false;
                }
            }
            else{
                cerr<<Vp[i].id<<"  out of index"<<endl;
                id_ok = false;
            }
        }
        rep(i,exist.size()) if(!exist[Vp[i].id]){
            ES(Vp[i].id) PL("not found")
            id_ok = false;
        }
        ac = vector<bool>(exist.size(), false);

        if(id_ok) cerr<<"id check ok!"<<endl;
        else cerr<<"id check not ok ...(-_-)..."<<endl;
    }

    void removeSamePoints(){
        ///--------------------座標重複排除-----------------------///
        cerr<<"original Vp size: "<<Vp.size()<<endl;
        sort(ALL(Vp));
        ERASE(Vp);
        cerr<<"after erase Vp size: "<<Vp.size()<<endl;
        cerr<<endl;
    }

    template<typename F>
    void extract(const vector<bool>& acceptMin, const vector<bool>& acceptMax, F isOK){
        ///--------------------外形抽出-----------------------///
        vector<double> abstX, abstY, abstZ;
        abstX.reserve((Mx-mx)/resolution);
        abstY.reserve((My-my)/resolution);
        abstZ.reserve((Mz-mz)/resolution);
        for(double x=mx; x<=Mx; x+=resolution) abstX.push_back(x);
        for(double y=my; y<=My; y+=resolution) abstY.push_back(y);
        for(double z=mz; z<=Mz; z+=resolution) abstZ.push_back(z);
        abstX.push_back(abstX.back()+resolution);
        abstY.push_back(abstY.back()+resolution);
        abstZ.push_back(abstZ.back()+resolution);
        EL(abstX.size())
        EL(abstY.size())
        EL(abstZ.size())
        cerr<<endl;
        
        string dir = "xyz";
        rep(i,3){

            if(acceptMin[i] || acceptMax[i]) cerr<<"remove vertex in direction "<<dir[i]<<endl;
            else continue;

            //i==2
            vector<double>* abst1;// = (i == 0 ? abstX : i == 1 ? abstY : abstZ);
            vector<double>* abst2;// = (i == 0 ? abstY : i == 1 ? abstZ : abstX);
            if(i==0){
                abst1 = &abstY;
                abst2 = &abstZ;
            }
            else if(i==1){
                abst1 = &abstZ;
                abst2 = &abstX;
            }
            else {
                abst1 = &abstX;
                abst2 = &abstY;
            }

            Vec minVecInit(0., 0., 0., -1); minVecInit.get(i)=1e10;
            Vec maxVecInit(0., 0., 0., -1); maxVecInit.get(i)=-1e10;

            vector<vector<Vec>> minV(abst1->size(), vector<Vec>(abst2->size(), minVecInit));
            vector<vector<Vec>> maxV(abst1->size(), vector<Vec>(abst2->size(), maxVecInit));
            rep(j, Vp.size()){
                int id1 = upper_bound(abst1->begin(), abst1->end(), Vp[j].get((i+1)%3)) - abst1->begin();
                int id2 = upper_bound(abst2->begin(), abst2->end(), Vp[j].get((i+2)%3)) - abst2->begin();
                if(minV[id1][id2].get(i)>Vp[j].get(i)) minV[id1][id2] = Vp[j];
                if(maxV[id1][id2].get(i)<Vp[j].get(i)) maxV[id1][id2] = Vp[j];
            }

            rep(j, abst1->size()) rep(k, abst2->size()){
                if(acceptMin[i] && minV[j][k].id!=-1) ac[minV[j][k].id] = true;
                if(acceptMax[i] && maxV[j][k].id!=-1) ac[maxV[j][k].id] = true;
            }
        }
        rep(i,Vp.size()) if(ac[Vp[i].id] && isOK(Vp[i])){
            resultVp.push_back(Vp[i]);
        }
    }



    //すべての点に一様な操作
    template<typename F>
    void forEachVp(F func){
        rep(i, Vp.size()){
            func(Vp[i]);
        }
    }

    //より多くの操作が必要な場合
    template<typename F>
    void operate(F func){
        func(resultVp);
    }

    void output(string pcdname, vector<unsigned int> RGBA = {255, 255, 255, 0}){
        ofstream ofs(pcdname);
        ofs << fixed << setprecision(8);
        ofs << "# .PCD v0.7 - Point Cloud Data file format" << endl;
        ofs<<"VERSION 0.7"<<endl;
        ofs<<"FIELDS x y z rgba"<<endl;
        ofs<<"SIZE 4 4 4 4"<<endl;
        ofs<<"TYPE F F F U"<<endl;
        ofs<<"COUNT 1 1 1 1"<<endl;
        ofs<<"WIDTH "<<resultVp.size()<<endl;
        ofs<<"HEIGHT 1"<<endl;
        ofs<<"VIEWPOINT 0 0 0 1 0 0 0"<<endl;
        ofs << "POINTS " << resultVp.size() << endl;
        ofs << "DATA ascii" << endl;

        vector<unsigned int> rgba = RGBA;
        NormalDistRandom ndrand(0.0, 20.);

        rep(i, resultVp.size()){
            rgba = RGBA;
            int rand = ndrand();
            rep(j,3){
                rgba[j] += rand;
                chmin(rgba[j], 255);
                chmax(rgba[j], 0);
            }
            unsigned int color = rgba[0]<<16 | rgba[1]<<8 | rgba[2] | rgba[3]<<24;
            ofs<<resultVp[i].x<<" "<<resultVp[i].y<<" "<<resultVp[i].z<<" "<<color<<endl;
        }

        ofs.close();
    }

};



/*
    ./a.out objname.obj pcdname 111110 R G B A
    (XxYyZz) 1:true 0:false

    ツールの軸とy軸一致．
    先端が原点，y軸正方向を向くように．
*/

int main(int argc, char** argv) {

    string objname = "sample_cube.obj"; //cin>>filename;
    string pcdname = "result.pcd";
    vector<unsigned int> RGBA = {230, 230, 230, 0};

    vector<bool> acceptMax = {false, false, false};
    vector<bool> acceptMin = {true, false, false};


    if(1<argc) objname = argv[1];
    if(2<argc) pcdname = argv[2];
    if(3<argc){
        for(int i=0;i<6;i++){
            if(i&1) acceptMin[i/2] = argv[3][i]-'0';
            else acceptMax[i/2] = argv[3][i]-'0';
        }
    }
    rep(i,4) if(4+i<argc) RGBA[i] = stoi(argv[4+i]);



    PCDMaker maker;
    double scale = 0.001;// m -> mm
    double resolution = 0.001;// 1mm

    maker.input(objname, scale);

    maker.calcMaxMin();
    //先端が原点に来るように
    maker.forEachVp([&](Vec& Vpi)->void {
        Vpi.z -= maker.Mz;
    });
    //z軸回り90deg回転
    maker.forEachVp([&](Vec& Vpi) {
        Vec tmp(Vpi);
        double angle = 90.0 * M_PI / 180.0;
        tmp.x = Vpi.x * cos(angle) - Vpi.y * sin(angle);
        tmp.y = Vpi.x * sin(angle) + Vpi.y * cos(angle);
        Vpi = tmp;
        });
    //x軸回り-90deg回転
    maker.forEachVp([&](Vec& Vpi) {
        Vec tmp(Vpi);
        double angle = -90.0 * M_PI / 180.0;
        tmp.y = Vpi.y * cos(angle) - Vpi.z * sin(angle);
        tmp.z = Vpi.y * sin(angle) + Vpi.z * cos(angle);
        Vpi = tmp;
        });



    maker.calc(resolution);
    maker.checkID();
    maker.removeSamePoints();


    //-------------------target-----------------------
    //外形を抽出
    acceptMax = { true, true, true };
    acceptMin = { true, false, false };
    maker.calcMaxMin();
    maker.extract(acceptMin, acceptMax, [&](const Vec& Vpi) {
        //if (Vpi.y <= -0.1) return false;
        //if (Vpi.z <= -0.005) return false;
        if (Vpi.y < -0.2) return false;
        return true;
        });
    //根本側を消す
    //maker.operate([&](vector<Vec>& resultVp) {
    //    rep(i, resultVp.size()) {
    //        if (resultVp[i].y <= -0.1) {
    //        //if (resultVp[i].y <= -0.06) {
    //            swap(resultVp[i], resultVp.back());
    //            resultVp.pop_back();
    //            i--;
    //        }
    //    }
    //   });    
    //save as target
    maker.output(pcdname + "_target.pcd", RGBA);



    //-------------------mask-----------------------
    //xz方向に太らせる
    maker.operate([&](vector<Vec>& resultVp){
        int n = resultVp.size();
        rep(i,n){
            //先端から2cm以上離れている点
            double g = 1.2;
            if(resultVp[i].y<=-0.03){
                rep(j, 5) {
                    resultVp.push_back(Vec(resultVp[i].x * g, resultVp[i].y, resultVp[i].z * g, -1));
                    g += 0.2;
                }
            }
        }
    });
    //save as mask
    maker.output(pcdname+"_mask.pcd", RGBA);






    //-------------------tip-----------------------
    maker.operate([&](vector<Vec>& resultVp){
        resultVp.clear();
        resultVp.push_back(Vec(0., 0., 0., -1));
    });
    maker.output(pcdname+"_tip.pcd", RGBA);


    return 0;
}


