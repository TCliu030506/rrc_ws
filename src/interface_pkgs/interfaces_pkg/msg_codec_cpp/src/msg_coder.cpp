#include "../include/msg_codec_cpp/msg_coder.hpp"

MsgCoder::CmdScript& MsgCoder::getCommandTypes() {
    static CmdScript instance = {
        {std::array<char, 2>{'X', 'X'}, {{"cmd", ValType::CHAR2}}}
    };
    return instance;
}

void MsgCoder::registerCommand(const std::string& cmd, const Keys& format) {
    auto& registry = getCommandTypes();
    std::array<char, 2> cmd_key = {cmd[0], cmd[1]};
    registry[cmd_key] = format;
}

void MsgCoder::registerCommands(const CmdScript& commands) {
    auto& registry = getCommandTypes();
    registry.insert(commands.begin(), commands.end());
}

size_t MsgCoder::getTypeSize(ValType type_symbol) {
    switch (type_symbol) {
        case ValType::CHAR2: return 2;      // 双字符
        case ValType::INT8: return 1;       // int8_t
        case ValType::INT16: return 2;      // int16_t
        case ValType::INT32: return 4;      // int32_t
        case ValType::INT64: return 8;      // int64_t
        case ValType::FLOAT: return 4;      // float
        case ValType::DOUBLE: return 8;     // double
        default:
            throw std::invalid_argument("Unsupported type symbol: " + std::string(1, static_cast<char>(type_symbol)));
    }
}

MsgCoder::WordValue MsgCoder::parseValue(const std::vector<char>& str, size_t& pos, ValType type_symbol) {
    switch (type_symbol) {
        case ValType::CHAR2: 
        {
            pos += 2;
            return std::array<char, 2>{str[pos - 2], str[pos - 1]};
        }
        case ValType::INT8:
        {
            pos += 1;
            return static_cast<int8_t>(str[pos - 1]);
        }
        case ValType::INT16: 
        {
            pos += 2;
            return static_cast<int16_t>(str[pos - 2] << 8 | str[pos - 1]);
        }
        case ValType::INT32:
        {
            int32_t value;
            std::memcpy(&value, &str[pos], sizeof(int32_t));
            pos += 4;
            return value;
        };
        case ValType::INT64:
        {
            int64_t value;
            std::memcpy(&value, &str[pos], sizeof(int64_t));
            pos += 8;
            return value;
        }
        case ValType::FLOAT:
        {
            float value;
            std::memcpy(&value, &str[pos], sizeof(float));
            pos += 4;
            return value;
        }
        case ValType::DOUBLE:
        {
            double value;
            std::memcpy(&value, &str[pos], sizeof(double));
            pos += 8;
            return value;
        }
        default:
            throw std::invalid_argument("Unsupported type symbol: " + std::string(1, static_cast<char>(type_symbol)));
    }
}

void MsgCoder::encodeValue(std::vector<char>& buffer, const WordValue& value, ValType type_symbol) {
    switch (type_symbol)
    {
    case ValType::CHAR2:
        {
            const std::array<char, 2>& arr = std::get<std::array<char, 2>>(value);
            buffer.insert(buffer.end(), arr.begin(), arr.end());
        }
        break;
    case ValType::INT8:
        {
            int8_t val = std::get<int8_t>(value);
            buffer.push_back(static_cast<char>(val));
        }
        break;
    case ValType::INT16:
        {
            int16_t val = std::get<int16_t>(value);
            const char* bytes = reinterpret_cast<const char*>(&val);
            buffer.insert(buffer.end(), bytes, bytes + sizeof(val));
        }
        break;
    case ValType::INT32:
        {
            int32_t val = std::get<int32_t>(value);
            const char* bytes = reinterpret_cast<const char*>(&val);
            buffer.insert(buffer.end(), bytes, bytes + sizeof(val));
        }
        break;
    case ValType::INT64:
        {
            int64_t val = std::get<int64_t>(value);
            const char* bytes = reinterpret_cast<const char*>(&val);
            buffer.insert(buffer.end(), bytes, bytes + sizeof(val));
        }
        break;
    case ValType::FLOAT:
        {
            float val = std::get<float>(value);
            const char* bytes = reinterpret_cast<const char*>(&val);
            buffer.insert(buffer.end(), bytes, bytes + sizeof(val));
        }
        break;
    case ValType::DOUBLE:
        {
            double val = std::get<double>(value);
            const char* bytes = reinterpret_cast<const char*>(&val);
            buffer.insert(buffer.end(), bytes, bytes + sizeof(val));
        }
        break;
    default:
        throw std::invalid_argument("Unknown type symbol: " + std::string(1, static_cast<char>(type_symbol)));
    }
}

std::vector<char> MsgCoder::msg_encode(const KeyValues& data) {
    auto& commandTypes = getCommandTypes();
    return MsgCoder::msg_encode(data, commandTypes);
}

std::vector<char> MsgCoder::msg_encode(const KeyValues& data, CmdScript cmd_script_) {
    if (data.find("cmd") == data.end()) {
        throw std::invalid_argument("'cmd' field is required");
    }
    
    auto cmd = std::get<std::array<char, 2>>(data.at("cmd"));
    auto& commandTypes = cmd_script_;
    if (commandTypes.find(cmd) == commandTypes.end()) {
        throw std::invalid_argument("Unknown cmd: " + std::string(cmd.begin(), cmd.end()));
    }

    std::vector<char> result;
    auto selected_keys = commandTypes[cmd];
    
    // 根据命令定义的字段顺序进行编码
    for (const auto& key : selected_keys) {
        ValType type_symbol = key.second;
        auto it = data.find(key.first);
        if (it == data.end()) {
            throw std::invalid_argument("Missing field: " + key.first);
        }
        encodeValue(result, it->second, type_symbol);
    }
    
    return result;
}


MsgCoder::KeyValues MsgCoder::msg_decode(const std::vector<char>& msg) {
    auto& commandTypes = getCommandTypes();
    return MsgCoder::msg_decode(msg,commandTypes);
}

MsgCoder::KeyValues MsgCoder::msg_decode(const std::vector<char>& msg, CmdScript cmd_script_) {
    if (msg.size() < 2) {
        throw std::invalid_argument("no cmd inside msg, msg_size: " + std::to_string(msg.size()));
    }

    // 提取cmd (前2字节)
    std::array<char, 2> cmd = {msg[0], msg[1]};
    auto& commandTypes = cmd_script_;
    if (commandTypes.find(cmd) == commandTypes.end()) {
        throw std::invalid_argument("Unknown cmd: " + std::string(cmd.begin(), cmd.end()));
    }

    auto selected_keys = commandTypes[cmd];

    KeyValues result;
    size_t current_pos = 0;

    for (const auto& pair : selected_keys) {
        const auto& key = pair.first;
        ValType type_symbol = pair.second;
        size_t type_size = getTypeSize(type_symbol);
        
        if (current_pos + type_size > msg.size()) {
            throw std::invalid_argument("msg too short, current_pos: " + std::to_string(current_pos)
             + ", type_size: " + std::to_string(type_size) 
             + ", has msg_size: " + std::to_string(msg.size()));
        }

        result[key] = parseValue(msg, current_pos, type_symbol);
    }

    // 检查是否有额外的字节
    if (current_pos != msg.size()) {
        throw std::invalid_argument("msg has extra bytes");
    }

    return result;
}