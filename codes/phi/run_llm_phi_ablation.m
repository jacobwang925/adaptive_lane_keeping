clear; clc; close all;

thisDir = fileparts(mfilename('fullpath'));   % .../codes/phi
addpath(thisDir);  % llm_phi_barrier_rules_text, phi_expr_passes_barrier_check

repoRoot = fileparts(fileparts(thisDir));    % repository root (parent of codes/)

%% 1. Load Environment Variables
env_str = fileread(fullfile(repoRoot, '.env'));
env_lines = splitlines(env_str);
for i=1:length(env_lines)
    line = strtrim(env_lines{i});
    if ~isempty(line) && ~startsWith(line, '#') && contains(line, '=')
        idx = strfind(line, '=');
        key = strtrim(line(1:idx(1)-1));
        val = strtrim(line(idx(1)+1:end));
        setenv(key, val);
    end
end

%% 2. Setup Parameters
models = ["gpt-4o-mini", "gpt-5.4-mini", "gemini-2.5-flash", "deepseek-chat"];
num_trials = 100;
emax_test = 3;

prompt = "You are a mathematical control theory expert designing safety barrier functions. " + ...
    llm_phi_barrier_rules_text() + ...
    "Return ONLY JSON in the exact shape: {""phi_expr"": ""your_expression_here""}";

% Base MATLAB variables for validation (no Symbolic Math Toolbox required)

%% 3. Main Loop
all_results = struct();

for m_idx = 1:length(models)
    model = models(m_idx);
    fprintf('\n=== Running ablation for model: %s ===\n', model);
    
    valid_count = 0;
    expressions = strings(num_trials, 1);
    is_valid_arr = false(num_trials, 1);
    
    for k = 1:num_trials
        fprintf('Trial %d/%d ... ', k, num_trials);
        
        resp_expr = "";
        try
            % -- Call LLM --
            if contains(model, "gpt")
                apiKey = getenv('OPENAI_API_KEY');
                url = 'https://api.openai.com/v1/chat/completions';
                headers = {'Authorization', ['Bearer ' apiKey]; 'Content-Type', 'application/json'};
                body = struct('model', model, ...
                    'messages', {{struct('role', 'user', 'content', prompt)}}, ...
                    'temperature', 0.9);
                opts = weboptions('RequestMethod', 'post', 'HeaderFields', headers, 'MediaType', 'application/json', 'Timeout', 60);
                res = webwrite(url, jsonencode(body), opts);
                raw_text = res.choices(1).message.content;
                
            elseif contains(model, "gemini")
                apiKey = getenv('GEMINI_API_KEY');
                url = "https://generativelanguage.googleapis.com/v1beta/models/" + model + ":generateContent?key=" + apiKey;
                body = struct("contents", { {struct("role", "user", "parts", { {struct("text", prompt)} })} }, ...
                              "generationConfig", struct("temperature", 0.9));
                opts = weboptions('MediaType', 'application/json', 'Timeout', 60);
                res = webwrite(url, body, opts);
                raw_text = res.candidates(1).content.parts(1).text;
                
            elseif contains(model, "deepseek")
                apiKey = getenv('DEEPSEEK_API_KEY');
                url = 'https://api.deepseek.com/v1/chat/completions';
                headers = {'Authorization', ['Bearer ' apiKey]; 'Content-Type', 'application/json'};
                body = struct('model', model, 'messages', {{struct('role', 'user', 'content', prompt)}}, 'temperature', 0.9);
                opts = weboptions('RequestMethod', 'post', 'HeaderFields', headers, 'MediaType', 'application/json', 'Timeout', 60);
                res = webwrite(url, jsonencode(body), opts);
                raw_text = res.choices(1).message.content;
            end
            pause(1); % Rate limiting delay
            
            % -- Parse JSON --
            raw_text = regexprep(raw_text, '```[a-zA-Z]*', '');
            raw_text = strrep(raw_text, '```', '');
            raw_text = strtrim(raw_text);
            match = regexp(raw_text, '{.*}', 'match');
            if ~isempty(match)
                S = jsondecode(match{1});
                resp_expr = string(S.phi_expr);
            else
                error('No JSON found');
            end
            
            expressions(k) = resp_expr;
            
            % -- Validate Math (same as codes/phi/run_llm_pipeline.m validate_phi via phi_expr_passes_barrier_check) --
            [ok, val_0, val_emax, errMsg] = phi_expr_passes_barrier_check(resp_expr, emax_test);
            if ok
                is_valid_arr(k) = true;
                valid_count = valid_count + 1;
                fprintf('Valid! (%s)\n', resp_expr);
            elseif strlength(errMsg) > 0
                fprintf('Error/Invalid Syntax: %s\n', errMsg);
            else
                fprintf('Invalid (Bounds). (%s)\n', resp_expr);
            end
            
        catch ME
            fprintf('Error/Invalid Syntax: %s\n', ME.message);
        end
    end
    
    modelStr = strrep(strrep(model, '-', '_'), '.', '_');
    all_results.(modelStr) = struct('expressions', expressions, 'is_valid', is_valid_arr, 'valid_rate', valid_count / num_trials);
end

%% 4. Save Results
mat_path = fullfile(repoRoot, 'LLM', 'llm_results', 'phi_ablation.mat');
d = fileparts(mat_path);
if ~isempty(d) && ~isfolder(d)
    mkdir(d);
end
save(mat_path, 'all_results', 'models', 'num_trials');
fprintf('Done. Saved %s (models, num_trials, all_results).\n', mat_path);
