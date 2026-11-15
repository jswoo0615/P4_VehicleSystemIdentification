#include "p4/io/ControllerModel.hpp"

#include <cmath>
#include <cctype>
#include <fstream>
#include <iterator>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

namespace p4::io {
namespace {
struct Json {
    enum class Type {Null, Boolean, Number, String, Array, Object} type=Type::Null;
    double number=0;
    std::string string;
    std::vector<Json> array;
    std::map<std::string,Json> object;
    const Json& at(const std::string& key) const {
        if(type!=Type::Object) throw std::runtime_error("Expected JSON object at "+key);
        const auto it=object.find(key);
        if(it==object.end()) throw std::runtime_error("Missing JSON key: "+key);
        return it->second;
    }
    double numeric() const {
        if(type!=Type::Number || !std::isfinite(number))
            throw std::runtime_error("Expected finite JSON number");
        return number;
    }
    const std::string& text() const {
        if(type!=Type::String) throw std::runtime_error("Expected JSON string");
        return string;
    }
};

class Parser {
public:
    explicit Parser(const std::string& source):s_(source) {}
    Json parse() {
        auto value=parseValue(0);space();
        if(i_!=s_.size()) throw std::runtime_error("Trailing JSON content");
        return value;
    }
private:
    const std::string& s_;
    std::size_t i_=0;
    void space() {while(i_<s_.size()&&std::isspace(static_cast<unsigned char>(s_[i_]))) ++i_;}
    bool take(char c) {space();if(i_<s_.size()&&s_[i_]==c){++i_;return true;}return false;}
    void expect(char c) {if(!take(c)) throw std::runtime_error("Malformed JSON");}
    void literal(const char* word) {
        for(const char* p=word;*p;++p) if(i_==s_.size()||s_[i_++]!=*p)
            throw std::runtime_error("Malformed JSON literal");
    }
    std::string parseString() {
        expect('"');std::string out;
        while(i_<s_.size()) {
            const char c=s_[i_++];
            if(c=='"') return out;
            if(static_cast<unsigned char>(c)<0x20) throw std::runtime_error("Invalid JSON string");
            if(c!='\\') {out+=c;continue;}
            if(i_==s_.size()) throw std::runtime_error("Invalid JSON escape");
            const char e=s_[i_++];
            switch(e) {
                case '"':out+='"';break;case '\\':out+='\\';break;case '/':out+='/';break;
                case 'b':out+='\b';break;case 'f':out+='\f';break;case 'n':out+='\n';break;
                case 'r':out+='\r';break;case 't':out+='\t';break;
                default:throw std::runtime_error("Unsupported JSON escape");
            }
        }
        throw std::runtime_error("Unterminated JSON string");
    }
    Json parseNumber() {
        space();const auto start=i_;
        if(i_<s_.size()&&s_[i_]=='-') ++i_;
        if(i_==s_.size()) throw std::runtime_error("Invalid JSON number");
        if(s_[i_]=='0') ++i_;
        else {
            if(!std::isdigit(static_cast<unsigned char>(s_[i_]))) throw std::runtime_error("Invalid JSON number");
            while(i_<s_.size()&&std::isdigit(static_cast<unsigned char>(s_[i_]))) ++i_;
        }
        if(i_<s_.size()&&s_[i_]=='.') {
            ++i_;const auto digits=i_;
            while(i_<s_.size()&&std::isdigit(static_cast<unsigned char>(s_[i_]))) ++i_;
            if(i_==digits) throw std::runtime_error("Invalid JSON fraction");
        }
        if(i_<s_.size()&&(s_[i_]=='e'||s_[i_]=='E')) {
            ++i_;if(i_<s_.size()&&(s_[i_]=='+'||s_[i_]=='-')) ++i_;
            const auto digits=i_;
            while(i_<s_.size()&&std::isdigit(static_cast<unsigned char>(s_[i_]))) ++i_;
            if(i_==digits) throw std::runtime_error("Invalid JSON exponent");
        }
        Json out;out.type=Json::Type::Number;
        try {out.number=std::stod(s_.substr(start,i_-start));}
        catch(const std::exception&) {throw std::runtime_error("Invalid JSON number");}
        if(!std::isfinite(out.number)) throw std::runtime_error("Nonfinite JSON number");
        return out;
    }
    Json parseValue(int depth) {
        if(depth>64) throw std::runtime_error("JSON nesting too deep");
        space();if(i_==s_.size()) throw std::runtime_error("Unexpected end of JSON");
        Json out;const char c=s_[i_];
        if(c=='{') {
            out.type=Json::Type::Object;++i_;
            if(take('}')) return out;
            do {
                const auto key=parseString();expect(':');
                if(!out.object.emplace(key,parseValue(depth+1)).second)
                    throw std::runtime_error("Duplicate JSON key: "+key);
            } while(take(','));
            expect('}');return out;
        }
        if(c=='[') {
            out.type=Json::Type::Array;++i_;
            if(take(']')) return out;
            do {out.array.push_back(parseValue(depth+1));} while(take(','));
            expect(']');return out;
        }
        if(c=='"') {out.type=Json::Type::String;out.string=parseString();return out;}
        if(c=='t') {literal("true");out.type=Json::Type::Boolean;return out;}
        if(c=='f') {literal("false");out.type=Json::Type::Boolean;return out;}
        if(c=='n') {literal("null");return out;}
        return parseNumber();
    }
};

std::string readText(const std::string& path) {
    std::ifstream input(path);
    if(!input) throw std::runtime_error("Cannot open JSON: "+path);
    return {std::istreambuf_iterator<char>(input),std::istreambuf_iterator<char>()};
}
double number(const Json& parent,const char* key) {return parent.at(key).numeric();}
void validate(const ControllerModel& model) {
    const auto& a=model.a;const auto& s=model.a_speed;const auto& b=model.b0;
    for(double x:{a.gain,a.tau,a.bias,s.wheelbase,s.kv,s.tau,s.bias,
                  b.cf_over_m,b.cr_over_m,b.iz_over_m,b.lf,b.lr})
        if(!std::isfinite(x)) throw std::runtime_error("Nonfinite controller coefficient");
    if(a.tau<=0||s.wheelbase<=0||s.kv<0||s.tau<=0||b.cf_over_m<=0||
       b.cr_over_m<=0||b.iz_over_m<=0||b.lf<=0||b.lr<=0||
       std::abs(b.lf+b.lr-s.wheelbase)>1e-10)
        throw std::runtime_error("Invalid controller coefficient");
}
}

ControllerModel loadControllerModelText(const std::string& json) {
    const auto root=Parser(json).parse();
    if(root.at("schema").text()!="p4.controller_model.v1")
        throw std::runtime_error("Unsupported controller model schema");
    const auto& models=root.at("models");
    if(models.object.count("B1")) throw std::runtime_error("B1 is not a controller export model");
    const auto& a=models.at("A").at("parameters");
    const auto& s=models.at("A-Speed").at("parameters");
    const auto& b=models.at("B0").at("parameters");
    ControllerModel out{{number(a,"K_r_per_s"),number(a,"tau_r_s"),number(a,"b_radps")},
        {number(s,"L_m"),number(s,"k_v_s2_per_m"),number(s,"tau_r_s"),number(s,"b_radps")},
        {number(b,"Cf_over_m_mps2_per_rad"),number(b,"Cr_over_m_mps2_per_rad"),
         number(b,"Iz_over_m_m2"),number(b,"lf_m"),number(b,"lr_m")}};
    if(models.at("B0").at("bank_lateral_acceleration").text()!="+9.80665*sin(bank)")
        throw std::runtime_error("Unsupported bank convention");
    validate(out);return out;
}

ControllerModel loadControllerModel(const std::string& path) {
    return loadControllerModelText(readText(path));
}

ControllerModel loadGolfBaselineForParity(const std::string& path) {
    const auto root=Parser(readText(path)).parse();
    const auto& a=root.at("model_a");
    const auto& s=root.at("model_a_speed");
    const auto& b=root.at("model_b0");
    ControllerModel out{{number(a,"K_per_s"),number(a,"tau_s"),number(a,"bias_rad_per_s")},
        {number(s,"nominal_wheelbase_m"),number(s,"kv_s2_per_m"),number(s,"tau_s"),
         number(s,"bias_rad_per_s")},
        {number(b,"cf_over_m_mps2_per_rad"),number(b,"cr_over_m_mps2_per_rad"),
         number(b,"iz_over_m_m2"),number(b,"lf_m"),number(b,"lr_m")}};
    validate(out);return out;
}
} // namespace p4::io
