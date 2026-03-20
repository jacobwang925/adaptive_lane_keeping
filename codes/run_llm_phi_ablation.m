clear; clc; close all;

%% 1. Load Environment Variables
env_str = fileread('../.env');
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
models = ["gpt-4o-mini", "gemini-2.5-flash", "deepseek-chat"];
num_trials = 100;
emax_test = 3;

prompt = ...
    "You are a mathematical control theory expert designing safety barrier functions. " + ...
    "Produce a unique safety barrier function expression in MATLAB syntax using variables 'e' (lateral error) and 'emax' (maximum allowed error). " + ...
    "The function phi(e, emax) must satisfy: " + ...
    "1) phi(0, emax) >= 0 (usually 1) " + ...
    "2) phi(emax, emax) <= 0 " + ...
    "3) phi(-emax, emax) <= 0 " + ...
    "4) Symmetric: phi(e) = phi(-e) " + ...
    "5) Monotonically decreasing as |e| increases from 0 to emax. " + ...
    "Here are some standard working examples: '1-(e/emax)^2', '1-(e/emax)^4', 'cos(pi*e/(2*emax))', or '1-abs(e/emax)'. " + ...
    "You may use these, but please prioritize returning highly varied and creative functional shapes that still satisfy the mathematical constraints perfectly (e.g., using exp, log, higher order polynomials, squishing functions, fractional powers, etc.) alongside the standard ones. " + ...
    "IMPORTANT: Before responding, mentally substitute e=0 and e=emax into your expression and verify that phi(0,emax)>=0 and phi(emax,emax)<=0. If your expression does not pass these checks, revise it until it does. " + ...
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
            
            % -- Validate Math --
            % Convert the string to an anonymous function
            % Only strip element-wise operators (.*, ./, .^) to scalar ops, preserve decimal points
            clean_expr = strrep(resp_expr, '.*', '*');
            clean_expr = strrep(clean_expr, './', '/');
            clean_expr = strrep(clean_expr, '.^', '^');
            func_str = strcat('@(e, emax) ', clean_expr); 
            phi_func = str2func(func_str);
            
            % Check value at 0
            val_0 = double(phi_func(0, emax_test));
            % Check value at emax
            val_emax = double(phi_func(emax_test, emax_test));
            
            if isreal(val_0) && isreal(val_emax) && (val_0 >= 0) && (val_emax <= 0)
                is_valid_arr(k) = true;
                valid_count = valid_count + 1;
                fprintf('Valid! (%s)\n', resp_expr);
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

%% 4. Save and Plot Results
save('../LLM/llm_results/phi_ablation.mat', 'all_results');

% --- PLOTTING ---
figure('Color', 'w', 'Position', [100, 100, 1200, 400]);
t = tiledlayout(1, 3, 'TileSpacing', 'Compact');

% We will group identical expressions (by string match) for diversity
for i = 1:length(models)
    modelStr = strrep(strrep(models(i), '-', '_'), '.', '_');
    dat = all_results.(modelStr);
    
    nexttile;
    valid_pct = dat.valid_rate * 100;
    
    % Group identical valid expressions
    valid_exprs = dat.expressions(dat.is_valid);
    [unique_exprs, ~, idx] = unique(valid_exprs);
    counts = accumarray(idx, 1);
    
    % Sort by frequency
    [counts, sortIdx] = sort(counts, 'descend');
    unique_exprs = unique_exprs(sortIdx);
    
    % Keep top 3 diverse types in label, lump rest as "Other"
    labels = string(counts); % start with count as label
    for j=1:length(counts)
        if j <= 3
            shortened = char(unique_exprs(j));
            if length(shortened) > 15
                shortened = [shortened(1:15) '...'];
            end
            labels(j) = sprintf('%s (%d)', shortened, counts(j));
        else
            labels(j) = "";
        end
    end
    if length(counts) > 3
        labels(4) = sprintf('Other diverse (%d)', sum(counts(4:end)));
        counts = [counts(1:3); sum(counts(4:end))];
        labels = labels(1:4);
    end
    
    if isempty(counts)
        pie([dat.valid_rate, 1 - dat.valid_rate], {'Valid', 'Invalid'});
        title(sprintf('%s\n(0%% Valid)', models(i)));
    else
        % Combine invalid with valid diverse slices
        pie_data = [counts; num_trials - sum(counts)];
        labels = [labels; sprintf('Invalid (%d)', num_trials - sum(counts))];
        p = pie(pie_data, labels);
        title(sprintf('%s\nValid: %.1f%%, Unique: %d', models(i), valid_pct, length(unique_exprs)));
    end
end
sgtitle('LLM Phi Expression Generation: Validity & Diversity', 'FontWeight', 'bold');
saveas(gcf, 'data_mpc/figs_mpc/phi_ablation_plot.png');
disp('Done. Saved to LLM/llm_results/phi_ablation.mat and data_mpc/figs_mpc/phi_ablation_plot.png');
