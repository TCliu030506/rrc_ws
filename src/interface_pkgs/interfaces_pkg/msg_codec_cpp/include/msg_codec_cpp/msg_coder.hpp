#ifndef MSG_CODER_HPP
#define MSG_CODER_HPP

#include <iostream>
#include <cstring>
#include <vector>
#include <variant>
#include <map>

class MsgCoder {
public:
    enum class ValType {
        INT8,      // int8_t
        INT16,     // int16_t
        INT32,     // int32_t
        INT64,     // int64_t
        FLOAT,     // float
        DOUBLE,    // double
        CHAR2,     // char[2]
    };

    // 支持的消息字段类型
    using WordValue = std::variant<
        std::array<char, 2>,    // 双字符   符号2   2字节
        int8_t,                 // 整数值   符号b   1字节
        int16_t,                // 整数值   符号h   2字节
        int32_t,                // 整数值   符号i   4字节
        int64_t,                // 整数值   符号q   8字节
        float,                  // 浮点值   符号f   4字节
        double>;                // 浮点值   符号d   8字节

    // 消息类型-值映射表
    using KeyValues = std::map<std::string, WordValue>;

    // 消息字段-类型符号映射表
    using Keys = std::map<std::string, ValType>;

    // 命令-指令类型映射表
    using CmdScript = std::map<std::array<char, 2>, Keys>;

    /* 编码：字典 -> 字节流
       输入参数data为字符串形式
       示例: {"cmd": std::array<char, 2>{'M', 'V'}, "id": "1", "pos": "1000", "vel": "500"}    */
    static std::vector<char> msg_encode(const KeyValues& data);

    /* 编码：字典 -> 字节流
       输入参数data为字符串形式, cmd_script_为指定的编码指令表
       示例: 
       data:{"cmd": std::array<char, 2>{'M', 'V'}, "id": "1", "pos": "1000", "vel": "500"}
       cmd_script_:{ {std::array<char, 2>{'X', 'X'}, {{"cmd", ValType::CHAR2}}} }    */
    static std::vector<char> msg_encode(const KeyValues& data, CmdScript cmd_script_);

    /* 解码：字节流 -> 字典
       输入参数为字节流
       输出参数为字典，键为字段名，值为字符串形式的字段值    */
    static KeyValues msg_decode(const std::vector<char>& msg);

    /* 解码：字节流 -> 字典
       输入参数为字节流,cmd_script_为指定的编码指令表
       输出参数为字典，键为字段名，值为字符串形式的字段值
       示例: cmd_script_:{ {std::array<char, 2>{'X', 'X'}, {{"cmd", ValType::CHAR2}}} }    */
    static KeyValues msg_decode(const std::vector<char>& msg, CmdScript cmd_script_);

    /* 注册新的命令类型
    示例：
        MsgCoder::registerCommand(std::string("CF"), {{"cmd", ValType::CHAR2}, {"id", ValType::INT8}, {"pos", ValType::INT64}, {"vel", ValType::INT64}});
        参考类型表：
            ValType::INT8   : int8_t                 (1 byte)
            ValType::INT16  : int16_t                (2 bytes)    
            ValType::INT32  : int32_t                (4 bytes)
            ValType::INT64  : int64_t                (8 bytes)
            ValType::FLOAT  : float                  (4 bytes)
            ValType::DOUBLE : double                 (8 bytes) 
            ValType::CHAR2  : std::array<char, 2>    (2 bytes)    */
    static void registerCommand(const std::string& cmd, const Keys& format);

    /* 注册新的命令类型
    示例：
        CmdScript new_commands = {
            {std::array<char, 2>{'C', 'F'}, {{"cmd", '2'}, {"id", 'b'}, {"pos", 'q'}, {"vel", 'q'}}},
            {std::array<char, 2>{'R', 'S'}, {{"cmd", '2'}, {"id", 'b'}, {"status", 'b'}}}    };
        MsgCoder::registerCommands(new_commands); */
    static void registerCommands(const CmdScript& commands);

private:
    // 命令类型注册接口, 存储命令及其字段类型映射关系,
    // 目前仅支持std::array<char, 2>、int8、int16、int32、int64、float、double类型
    // 需要新的类型时，需同时修改编码器映射表和解码器映射表，并新增该类型的编码和解码函数
    // 每个命令的第一个字段必须是cmd，类型为std::array<char, 2>
    // 示例: {std::array<char, 2>{'M', 'V'}, {{"cmd", ValType::CHAR2}, {"id", ValType::INT8}, {"pos", ValType::INT64}, {"vel", ValType::INT64}}}
    static CmdScript& getCommandTypes();
    // 获取类型对应的字节大小
    static size_t getTypeSize(ValType type_symbol);
    // 解析字节流中的值
    static WordValue parseValue(const std::vector<char>& str, size_t& pos, ValType type_symbol);
    // 编码值到字节流
    static void encodeValue(std::vector<char>& buffer, const WordValue& value, ValType type_symbol);
};

#endif // MSG_CODER_HPP